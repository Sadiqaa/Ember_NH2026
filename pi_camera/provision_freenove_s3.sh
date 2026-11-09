#!/bin/bash
# One-shot, guarded Freenove ESP32-S3 provisioner for the Raspberry Pi.

set -Eeuo pipefail

readonly CAMERA_SERVICE="nathacks-camera.service"
readonly TOOL_ROOT="/home/nathacks/camera-oled/tools"
readonly IMAGE_ROOT="/home/nathacks/camera-oled/firmware/freenove_s3_i2c_ssd1306_mirror"
readonly BACKUP_ROOT="/home/nathacks/camera-oled/firmware/backups"
readonly ESPTOOL=(env "PYTHONPATH=$TOOL_ROOT" python3 -m esptool)
readonly EXPECTED_VENDOR="1a86"
readonly EXPECTED_PRODUCT="55d3"
readonly EXPECTED_MAC="68:ee:8f:4e:ae:08"
readonly EXPECTED_FLASH_LABEL="16MB"
readonly EXPECTED_FLASH_BYTES=16777216
readonly EXPECTED_FLASH_READ_LENGTH="0x1000000"
readonly EXPECTED_FLASH_SIZE_CODE=4
readonly EXPECTED_FLASH_MANUFACTURER="46"
readonly EXPECTED_FLASH_DEVICE="4018"
readonly EXPECTED_RECOVERY_SHA256="cf3235cf5dc18c1d788d07a9b7e8fc987c4eaca15c897e9343e25d192d59797b"
readonly WAIT_SECONDS=14400
readonly REUSE_BACKUP="${NATHACKS_REUSE_BACKUP:-}"

camera_was_stopped=false
backup_partial=""
backup=""

log() {
  printf '[%s] %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*"
}

restart_camera() {
  if [[ "$camera_was_stopped" == true ]]; then
    systemctl start "$CAMERA_SERVICE" || true
  fi
}

cleanup() {
  local status=$?
  if [[ -n "$backup_partial" && -f "$backup_partial" ]]; then
    rm -f -- "$backup_partial"
  fi
  restart_camera
  return "$status"
}

trap cleanup EXIT

find_ch343() {
  local device properties vendor product
  for device in /dev/ttyACM* /dev/ttyUSB* /dev/ttyCH343USB*; do
    [[ -c "$device" ]] || continue
    properties=$(udevadm info --query=property --name="$device" 2>/dev/null || true)
    vendor=$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$properties" | head -n 1)
    product=$(sed -n 's/^ID_MODEL_ID=//p' <<<"$properties" | head -n 1)
    if [[ "${vendor,,}" == "$EXPECTED_VENDOR" && "${product,,}" == "$EXPECTED_PRODUCT" ]]; then
      printf '%s\n' "$device"
      return 0
    fi
  done
  return 1
}

require_firmware_hashes() {
  cd "$IMAGE_ROOT"
  sha256sum --check --strict <<'CHECKSUMS'
f94c5d786a7a8fab06ac5d10e33bf37711a6697636dc037559ea19cc410a17f0  boot_app0.bin
2a71d69b471e20c2bac7fb469f3c6a807b3ebee780e348e5889db0da849ca363  bootloader.bin
3513a44117a9afbd8d9119bdf760267f19a53e9c2d8cc6c8026e996a6d03cf6f  firmware.bin
bd0f7954aca2ef7d925ee21aaa1f3dc8822d1d6ce5cbbd26a135e5886bfff6ce  partitions.bin
CHECKSUMS
}

validate_staged_layout() {
  env \
    IMAGE_ROOT="$IMAGE_ROOT" \
    EXPECTED_FLASH_BYTES="$EXPECTED_FLASH_BYTES" \
    EXPECTED_FLASH_SIZE_CODE="$EXPECTED_FLASH_SIZE_CODE" \
    python3 - <<'PY'
import os
import struct
from pathlib import Path

root = Path(os.environ["IMAGE_ROOT"])
flash_bytes = int(os.environ["EXPECTED_FLASH_BYTES"])
flash_size_code = int(os.environ["EXPECTED_FLASH_SIZE_CODE"])


def fail(message: str) -> None:
    raise SystemExit(f"staged image validation failed: {message}")


# Both ESP images must advertise the connected target's 16 MB capacity. Mode
# 2 is DIO, matching the conservative write setting used below.
for name in ("bootloader.bin", "firmware.bin"):
    data = (root / name).read_bytes()
    if len(data) < 4 or data[0] != 0xE9:
        fail(f"{name} is not an ESP image")
    if data[2] != 2:
        fail(f"{name} flash mode is {data[2]}, expected DIO (2)")
    if data[3] >> 4 != flash_size_code:
        fail(
            f"{name} flash-size code is {data[3] >> 4}, "
            f"expected {flash_size_code} for 16 MB"
        )

partition_path = root / "partitions.bin"
partition_data = partition_path.read_bytes()
if len(partition_data) > 0x1000:
    fail("partition table does not fit the 0x8000..0x8fff region")

entries = []
for offset in range(0, len(partition_data), 32):
    chunk = partition_data[offset : offset + 32]
    if len(chunk) < 32:
        fail("truncated partition-table entry")
    magic = struct.unpack_from("<H", chunk)[0]
    if magic in (0xFFFF, 0xEBEB):
        break
    if magic != 0x50AA:
        fail(f"bad partition magic 0x{magic:04x} at table offset 0x{offset:x}")
    _magic, part_type, subtype, part_offset, size, raw_label, _flags = (
        struct.unpack("<HBBII16sI", chunk)
    )
    label = raw_label.split(b"\0", 1)[0].decode("ascii", "strict")
    if not size or part_offset < 0x9000 or part_offset + size > flash_bytes:
        fail(f"partition {label!r} is outside the 16 MB target")
    entries.append((part_offset, part_offset + size, part_type, subtype, label, size))

if not entries:
    fail("partition table contains no partitions")
for previous, current in zip(sorted(entries), sorted(entries)[1:]):
    if current[0] < previous[1]:
        fail(f"partitions {previous[4]!r} and {current[4]!r} overlap")
if max(entry[1] for entry in entries) != flash_bytes:
    fail("partition layout does not span the detected 16 MB capacity")

app0 = next(
    (entry for entry in entries if entry[2:5] == (0, 0x10, "app0")), None
)
if app0 is None or app0[0] != 0x10000:
    fail("missing OTA app0 partition at 0x10000")
if (root / "firmware.bin").stat().st_size > app0[5]:
    fail("firmware.bin does not fit the app0 partition")

otadata = next(
    (entry for entry in entries if entry[2:5] == (1, 0, "otadata")), None
)
if otadata is None or otadata[0] != 0xE000:
    fail("missing otadata partition at 0xe000")
if (root / "boot_app0.bin").stat().st_size > otadata[5]:
    fail("boot_app0.bin does not fit the otadata partition")
if (root / "bootloader.bin").stat().st_size > 0x8000:
    fail("bootloader.bin overlaps the partition table")

print(
    f"staged layout OK: 16 MB, {len(entries)} partitions, "
    f"app0={app0[5]} bytes"
)
PY
}

esptool_at() {
  local device=$1
  local baud=$2
  shift 2
  "${ESPTOOL[@]}" --chip esp32s3 --port "$device" --baud "$baud" \
    --before default_reset --after hard_reset "$@"
}

log "validating the staged firmware before accepting a device"
require_firmware_hashes
validate_staged_layout

log "waiting up to $WAIT_SECONDS seconds for Freenove CH343 USB-UART ($EXPECTED_VENDOR:$EXPECTED_PRODUCT)"
device=""
waited=0
while (( waited < WAIT_SECONDS )); do
  udevadm settle --timeout=2 || true
  if device=$(find_ch343); then
    break
  fi
  sleep 1
  ((waited += 1))
done

if [[ -z "$device" ]]; then
  log "ERROR: no Freenove CH343 USB-UART appeared"
  exit 10
fi
log "found $device"

systemctl stop "$CAMERA_SERVICE"
camera_was_stopped=true
sleep 1

baud=460800
chip_log=$(mktemp)
esptool_at "$device" "$baud" chip_id 2>&1 | tee "$chip_log"
grep -q 'ESP32-S3' "$chip_log" || {
  log "ERROR: esptool did not identify an ESP32-S3"
  exit 11
}
grep -qi "^MAC:[[:space:]]*$EXPECTED_MAC$" "$chip_log" || {
  log "ERROR: ESP32-S3 MAC does not match the authorized physical board"
  exit 11
}

flash_log=$(mktemp)
esptool_at "$device" "$baud" flash_id 2>&1 | tee "$flash_log"
grep -Eq "Detected flash size:[[:space:]]*$EXPECTED_FLASH_LABEL|Flash size:[[:space:]]*$EXPECTED_FLASH_LABEL" "$flash_log" || {
  log "ERROR: target did not report $EXPECTED_FLASH_LABEL flash"
  exit 12
}
grep -Eq "^Manufacturer:[[:space:]]*$EXPECTED_FLASH_MANUFACTURER$" "$flash_log" &&
  grep -Eq "^Device:[[:space:]]*$EXPECTED_FLASH_DEVICE$" "$flash_log" || {
    log "ERROR: flash JEDEC identity does not match the authorized 16MB target"
    exit 12
  }

security_log=$(mktemp)
esptool_at "$device" "$baud" get_security_info 2>&1 | tee "$security_log"
security_flags_hex=$(sed -n 's/^Flags: \(0x[0-9a-fA-F]*\).*/\1/p' "$security_log" | head -n 1)
crypt_count_hex=$(sed -n 's/^Flash_Crypt_Cnt: \(0x[0-9a-fA-F]*\).*/\1/p' "$security_log" | head -n 1)
[[ -n "$security_flags_hex" && -n "$crypt_count_hex" ]] || {
  log "ERROR: could not parse the ESP32-S3 security state"
  exit 13
}
security_flags=$((security_flags_hex))
crypt_count=$((crypt_count_hex))
crypt_bits=$(( (crypt_count & 1) + ((crypt_count >> 1) & 1) + ((crypt_count >> 2) & 1) ))

secure_boot_log=$(mktemp)
# esptool 4.5 exposes arbitrary register reads through the read_mem CLI
# operation (its Python target method is still named read_reg).
esptool_at "$device" "$baud" read_mem 0x60007038 2>&1 | tee "$secure_boot_log"
secure_boot_hex=$(sed -n 's/^0x60007038 = \(0x[0-9a-fA-F]*\)$/\1/p' "$secure_boot_log" | tail -n 1)
[[ -n "$secure_boot_hex" ]] || {
  log "ERROR: could not read the secure-boot eFuse register"
  exit 13
}
secure_boot_reg=$((secure_boot_hex))

if (( (security_flags & 4) != 0 || (crypt_bits & 1) != 0 || (secure_boot_reg & 0x100000) != 0 )); then
  log "ERROR: secure download, flash encryption, or secure boot is enabled; refusing to overwrite"
  exit 13
fi

mkdir -p "$BACKUP_ROOT"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
preflight_log="$BACKUP_ROOT/freenove_s3_preflight_${stamp}.log"
{
  printf '%s\n' '=== CHIP ==='
  cat "$chip_log"
  printf '%s\n' '=== FLASH ==='
  cat "$flash_log"
  printf '%s\n' '=== SECURITY ==='
  cat "$security_log"
  printf '%s\n' '=== SECURE BOOT EFUSE REGISTER ==='
  cat "$secure_boot_log"
} > "$preflight_log"
chown nathacks:nathacks "$preflight_log"

if [[ -n "$REUSE_BACKUP" ]]; then
  backup=$(readlink -f -- "$REUSE_BACKUP")
  [[ "$backup" == "$BACKUP_ROOT/"* && -f "$backup" ]] || {
    log "ERROR: reusable backup must be a regular file inside $BACKUP_ROOT"
    exit 14
  }
  [[ $(stat -c '%s' "$backup") == "$EXPECTED_FLASH_BYTES" ]] || {
    log "ERROR: reusable recovery image is not exactly $EXPECTED_FLASH_LABEL"
    exit 14
  }
  printf '%s  %s\n' "$EXPECTED_RECOVERY_SHA256" "$backup" | sha256sum --check --strict || {
    log "ERROR: reusable recovery image does not match the verified original backup"
    exit 14
  }
  log "reusing the verified original full-chip backup: $backup"
  sync "$backup" "$backup.sha256" "$preflight_log"
else
  backup_partial="$BACKUP_ROOT/freenove_s3_16mb_before_${stamp}.bin.partial"
  backup="$BACKUP_ROOT/freenove_s3_16mb_before_${stamp}.bin"
  available_bytes=$(df --output=avail -B1 "$BACKUP_ROOT" | tail -n 1 | tr -d '[:space:]')
  [[ "$available_bytes" =~ ^[0-9]+$ ]] &&
    (( available_bytes >= EXPECTED_FLASH_BYTES + 1048576 )) || {
      log "ERROR: insufficient free space for a complete 16MB backup"
      exit 14
    }
  log "reading the complete $EXPECTED_FLASH_LABEL flash backup"
  esptool_at "$device" "$baud" read_flash 0x0 "$EXPECTED_FLASH_READ_LENGTH" "$backup_partial"
  [[ $(stat -c '%s' "$backup_partial") == "$EXPECTED_FLASH_BYTES" ]] || {
    log "ERROR: flash backup is not exactly $EXPECTED_FLASH_LABEL"
    exit 14
  }
  mv "$backup_partial" "$backup"
  chown nathacks:nathacks "$backup"
  sha256sum "$backup" | tee "$backup.sha256"
  chown nathacks:nathacks "$backup.sha256"
  # Make the complete recovery image and its evidence durable before the first
  # flash-sector write is allowed to begin.
  sync "$backup" "$backup.sha256" "$preflight_log"
fi

log "writing the verified OLED + flame + optional MPU6050 sensor firmware"
cd "$IMAGE_ROOT"
esptool_at "$device" "$baud" write_flash -z \
  --flash_mode dio --flash_freq 80m --flash_size "$EXPECTED_FLASH_LABEL" \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0xe000 boot_app0.bin \
  0x10000 firmware.bin

log "verifying every flashed region"
esptool_at "$device" "$baud" verify_flash --diff yes \
  0x0000 bootloader.bin \
  0x8000 partitions.bin \
  0xe000 boot_app0.bin \
  0x10000 firmware.bin

log "capturing the receiver boot log"
monitor_log="$BACKUP_ROOT/freenove_s3_after_${stamp}.log"
env DEVICE="$device" MONITOR_LOG="$monitor_log" python3 - <<'PY'
import os
import time

import serial

device = os.environ["DEVICE"]
path = os.environ["MONITOR_LOG"]
deadline = time.monotonic() + 8.0
data = bytearray()
with serial.Serial(device, 921600, timeout=0.25) as stream:
    stream.dtr = False
    stream.rts = False
    while time.monotonic() < deadline:
        data.extend(stream.read(4096))
text = data.decode("utf-8", "replace")
with open(path, "w", encoding="utf-8") as output:
    output.write(text)
print(text, end="")
PY
chown nathacks:nathacks "$monitor_log"

if grep -q 'STOP: OLED' "$monitor_log"; then
  log "WARNING: firmware is valid but the OLED did not acknowledge on I2C"
elif grep -q 'READY VSR1' "$monitor_log"; then
  log "receiver booted and the OLED acknowledged"
else
  log "WARNING: no READY/STOP line was captured; service-level verification is still required"
fi

log "provisioning completed; backup=$backup"
