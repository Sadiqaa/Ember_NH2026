#!/usr/bin/env python3
"""Dependency-light firefighter HUD for the shared Pi camera bridge.

The module accepts the text telemetry format used by ``visor_display.py`` and
renders it both onto an RGB camera frame and into a 128x64 SSD1306 payload.  It
does not own the camera or the OLED serial port.
"""

from __future__ import annotations

import glob
import logging
import math
import os
import re
import socket
import sqlite3
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional, Sequence

try:
    import cv2
except ModuleNotFoundError:  # Optional on development Macs and in unit tests.
    cv2 = None


NUM_SLICES = 36
FADE_PER_30_FPS_FRAME = 0.998
STRIP_SPAN = 120
CAMERA_HORIZONTAL_HALF_FOV = 31.0
THERMAL_ELLIPSE_BANDS = 9
# A confirmed forward fire needs a strong calibrated response.  Live target
# history showed real emitter locks above 3000, so this higher threshold keeps
# ordinary reflections from becoming FIRE locks.
FLAME_HOT = 2800
FACING_ANGLE = 15
GAS_WARMUP_S = 150
GAS_MARGIN = 300
SENSOR_FRESH_S = 2.5
DIRECTION_PAIRING_S = 1.0
OLED_LINK_FRESH_S = 5.0
IR_ADC_MAX = 4095.0
IR_CALIBRATION_MIN_SPAN = 64.0
IR_MODE_UNAVAILABLE = "unavailable"
IR_MODE_CALIBRATION_NEEDED = "calibration_needed"
IR_MODE_SCAN = "scan"
IR_MODE_LOCK = "lock"
IR_LOCK_RELEASE = FLAME_HOT * 0.75
FIRE_CONFIRM_SAMPLES = 3
TARGET_LOG_COOLDOWN_S = 5.0
GUIDED_SCAN_EMA_ALPHA = 0.30
# Ignore weak reflections and ambient IR until the calibrated signal reaches a
# meaningful fraction of the sensor's range.  This was deliberately raised
# from 600 after live testing showed that small fluctuations made the visor
# begin turn guidance too readily.
GUIDED_SCAN_MIN_PEAK = 1200.0
GUIDED_SCAN_CONFIRM_SAMPLES = 2
GUIDED_SCAN_RELEASE_SAMPLES = 8
GUIDED_SCAN_DROP_COUNTS = 220.0
GUIDED_SCAN_DROP_RATIO = 0.15
GUIDED_SCAN_DROP_SAMPLES = 3
GUIDED_SCAN_ALTERNATE_S = 5.0


LOG = logging.getLogger("visor-hud")


# Five-bit rows, seven rows per character. Only uppercase is rendered.
FONT_5X7 = {
    " ": (0, 0, 0, 0, 0, 0, 0),
    "-": (0, 0, 0, 31, 0, 0, 0),
    ".": (0, 0, 0, 0, 0, 12, 12),
    ":": (0, 12, 12, 0, 12, 12, 0),
    "/": (1, 2, 4, 8, 16, 0, 0),
    "?": (14, 17, 1, 2, 4, 0, 4),
    "0": (14, 17, 19, 21, 25, 17, 14),
    "1": (4, 12, 4, 4, 4, 4, 14),
    "2": (14, 17, 1, 2, 4, 8, 31),
    "3": (30, 1, 1, 14, 1, 1, 30),
    "4": (2, 6, 10, 18, 31, 2, 2),
    "5": (31, 16, 16, 30, 1, 1, 30),
    "6": (14, 16, 16, 30, 17, 17, 14),
    "7": (31, 1, 2, 4, 8, 8, 8),
    "8": (14, 17, 17, 14, 17, 17, 14),
    "9": (14, 17, 17, 15, 1, 1, 14),
    "A": (14, 17, 17, 31, 17, 17, 17),
    "B": (30, 17, 17, 30, 17, 17, 30),
    "C": (14, 17, 16, 16, 16, 17, 14),
    "D": (30, 17, 17, 17, 17, 17, 30),
    "E": (31, 16, 16, 30, 16, 16, 31),
    "F": (31, 16, 16, 30, 16, 16, 16),
    "G": (14, 17, 16, 23, 17, 17, 15),
    "H": (17, 17, 17, 31, 17, 17, 17),
    "I": (14, 4, 4, 4, 4, 4, 14),
    "J": (7, 2, 2, 2, 18, 18, 12),
    "K": (17, 18, 20, 24, 20, 18, 17),
    "L": (16, 16, 16, 16, 16, 16, 31),
    "M": (17, 27, 21, 21, 17, 17, 17),
    "N": (17, 25, 21, 19, 17, 17, 17),
    "O": (14, 17, 17, 17, 17, 17, 14),
    "P": (30, 17, 17, 30, 16, 16, 16),
    "Q": (14, 17, 17, 17, 21, 18, 13),
    "R": (30, 17, 17, 30, 20, 18, 17),
    "S": (15, 16, 16, 14, 1, 1, 30),
    "T": (31, 4, 4, 4, 4, 4, 4),
    "U": (17, 17, 17, 17, 17, 17, 14),
    "V": (17, 17, 17, 17, 17, 10, 4),
    "W": (17, 17, 17, 21, 21, 21, 10),
    "X": (17, 17, 10, 4, 10, 17, 17),
    "Y": (17, 17, 10, 4, 4, 4, 4),
    "Z": (31, 1, 2, 4, 8, 16, 31),
}


SENSOR_FIELDS = frozenset(("HEADING", "AIR", "FLAME", "GAS"))
SENSOR_RANGES = {
    "HEADING": (0.0, 360.0),
    "AIR": (-100.0, 1000.0),
    "FLAME": (0.0, 4095.0),
    "GAS": (0.0, 4095.0),
}

OLED_COUNTER_ALIASES = {
    "OK": "accepted_frames",
    "ACCEPTED": "accepted_frames",
    "SHOWN": "displayed_frames",
    "DISPLAYED": "displayed_frames",
    "CRC": "crc_errors",
    "LEN": "length_errors",
    "LENGTH": "length_errors",
    "PARTIAL": "packet_timeouts",
    "TIMEOUTS": "packet_timeouts",
    "LAST": "last_sequence",
}
OLED_COUNTER_PATTERN = re.compile(
    r"\b(" + "|".join(OLED_COUNTER_ALIASES) + r")\s*[=:]\s*(\d+)\b",
    re.IGNORECASE,
)


def parse_line(line: str) -> dict[str, float]:
    """Parse one HEADING/AIR/FLAME/GAS line, ignoring malformed values."""

    values: dict[str, float] = {}
    for part in line.split(","):
        if ":" not in part:
            continue
        name, raw = part.split(":", 1)
        name = name.strip().upper()
        if name not in SENSOR_FIELDS:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        lower, upper = SENSOR_RANGES[name]
        if not math.isfinite(value) or not lower <= value <= upper:
            continue
        values[name] = 0.0 if name == "HEADING" and value == 360.0 else value
    return values


def parse_oled_status(line: str) -> dict[str, int]:
    """Parse receiver counters without treating arbitrary boot text as status.

    The live firmware emits ``VSRSTAT`` while frames are arriving.  The older
    timeout diagnostic uses the same counter names in a ``Camera timeout``
    sentence, so accepting both formats keeps post-flash diagnostics useful
    across firmware revisions.
    """

    normalized = line.strip()
    if not (
        normalized.upper().startswith("VSRSTAT")
        or normalized.lower().startswith("camera timeout")
    ):
        return {}
    values: dict[str, int] = {}
    for match in OLED_COUNTER_PATTERN.finditer(normalized):
        source = match.group(1).upper()
        value = int(match.group(2))
        target = OLED_COUNTER_ALIASES[source]
        if target == "last_sequence":
            if value <= 0xFFFF:
                values[target] = value
        elif value <= 0xFFFFFFFF:
            values[target] = value
    return values


def wrap_angle(value: float) -> float:
    return (value + 540.0) % 360.0 - 180.0


def normalize_ir_strength(
    raw: float,
    ambient_raw: float,
    target_raw: float,
) -> float:
    """Map a two-point IR calibration onto 0..4095 target strength.

    ``ambient_raw`` is captured with the emitter off/receiver covered and
    ``target_raw`` with the emitter on and aimed at the receiver.  The signed
    span therefore infers active-high versus active-low automatically.
    Values beyond either calibration endpoint are safely clamped.
    """

    values = (raw, ambient_raw, target_raw)
    if any(not math.isfinite(value) or not 0.0 <= value <= IR_ADC_MAX for value in values):
        raise ValueError("IR calibration values must be finite 12-bit ADC readings")
    span = target_raw - ambient_raw
    if abs(span) < IR_CALIBRATION_MIN_SPAN:
        raise ValueError(
            f"IR calibration span must be at least {IR_CALIBRATION_MIN_SPAN:.0f} counts"
        )
    fraction = (raw - ambient_raw) / span
    return max(0.0, min(IR_ADC_MAX, fraction * IR_ADC_MAX))


def classify_ir_target(
    *,
    fresh: bool,
    calibrated: bool,
    raw: Optional[float],
    strength: Optional[float],
) -> str:
    """Return the wearer-facing state shared by every display.

    The raw ADC value is useful for diagnostics, but it cannot be interpreted
    until a two-point calibration establishes the receiver's polarity and
    response. A single forward-facing receiver can then truthfully report
    either ``scan`` or ``lock``; left/right guidance still requires a heading-
    tagged scan map.
    """

    if not fresh or raw is None:
        return IR_MODE_UNAVAILABLE
    if not calibrated:
        return IR_MODE_CALIBRATION_NEEDED
    if strength is None:
        return IR_MODE_UNAVAILABLE
    return IR_MODE_LOCK if strength >= FLAME_HOT else IR_MODE_SCAN


@dataclass(frozen=True)
class TargetSighting:
    """One persistent IR-target lock event.

    ``bearing`` is relative yaw only when the optional heading sensor was
    fresh at detection time.  A null bearing means the receiver confirmed the
    target straight ahead but no position axis was available; it must never be
    presented as a map coordinate.
    """

    seen_at_utc: str
    ir_raw: float
    ir_strength: float
    heading: Optional[float]
    bearing: Optional[float]

    @property
    def has_relative_bearing(self) -> bool:
        return self.bearing is not None


class TargetHistory:
    """Small SQLite store for target-lock transitions."""

    def __init__(self, path: str) -> None:
        if not path:
            raise ValueError("target history path cannot be empty")
        self.path = os.path.abspath(os.path.expanduser(path))
        directory = os.path.dirname(self.path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        self._initialize()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=2.0)
        connection.execute("PRAGMA busy_timeout=2000")
        return connection

    def _initialize(self) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS target_sightings (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    seen_at_utc TEXT NOT NULL,
                    ir_raw REAL NOT NULL,
                    ir_strength REAL NOT NULL,
                    heading_deg REAL,
                    bearing_deg REAL,
                    CHECK (ir_raw >= 0 AND ir_raw <= 4095),
                    CHECK (ir_strength >= 0 AND ir_strength <= 4095),
                    CHECK (heading_deg IS NULL OR (heading_deg >= 0 AND heading_deg < 360)),
                    CHECK (bearing_deg IS NULL OR (bearing_deg >= 0 AND bearing_deg < 360))
                )
                """
            )

    @staticmethod
    def _validate(sighting: TargetSighting) -> None:
        values = (sighting.ir_raw, sighting.ir_strength)
        if any(not math.isfinite(value) or not 0 <= value <= IR_ADC_MAX for value in values):
            raise ValueError("target IR values must be finite 12-bit readings")
        for name, value in (("heading", sighting.heading), ("bearing", sighting.bearing)):
            if value is not None and (not math.isfinite(value) or not 0 <= value < 360):
                raise ValueError(f"target {name} must be from 0 up to 360 degrees")

    def record(self, sighting: TargetSighting) -> TargetSighting:
        self._validate(sighting)
        with self._connect() as connection:
            connection.execute(
                """
                INSERT INTO target_sightings
                    (seen_at_utc, ir_raw, ir_strength, heading_deg, bearing_deg)
                VALUES (?, ?, ?, ?, ?)
                """,
                (
                    sighting.seen_at_utc,
                    sighting.ir_raw,
                    sighting.ir_strength,
                    sighting.heading,
                    sighting.bearing,
                ),
            )
        return sighting

    def latest(self) -> Optional[TargetSighting]:
        with self._connect() as connection:
            row = connection.execute(
                """
                SELECT seen_at_utc, ir_raw, ir_strength, heading_deg, bearing_deg
                FROM target_sightings
                ORDER BY id DESC
                LIMIT 1
                """
            ).fetchone()
        if row is None:
            return None
        return TargetSighting(
            seen_at_utc=str(row[0]),
            ir_raw=float(row[1]),
            ir_strength=float(row[2]),
            heading=None if row[3] is None else float(row[3]),
            bearing=None if row[4] is None else float(row[4]),
        )

    def count(self) -> int:
        with self._connect() as connection:
            row = connection.execute("SELECT COUNT(*) FROM target_sightings").fetchone()
        return int(row[0]) if row is not None else 0


@dataclass(frozen=True)
class HudSnapshot:
    heading: Optional[float]
    air: Optional[float]
    flame: Optional[float]
    gas: Optional[float]
    gas_baseline: Optional[float]
    slices: tuple[float, ...]
    fresh: bool
    demo: bool
    ir_calibrated: bool
    ir_raw: Optional[float]
    ir_strength: Optional[float]
    fire_confirmed: bool
    last_target: Optional[TargetSighting]
    session_fire_bearing: Optional[float]
    session_fire_strength: Optional[float]
    guided_scan: "GuidedScanGuidance"


@dataclass(frozen=True)
class GuidedScanGuidance:
    """Instructional search state for one receiver without a heading axis.

    ``left`` and ``right`` mean "follow this commanded sweep", not a measured
    target bearing.  Measured bearings continue to come only from
    :func:`find_ir_target_vector`.
    """

    state: str
    instruction: str
    strength: Optional[float]
    peak_strength: Optional[float]
    signal_confirmed: bool = False
    measured_bearing: bool = False
    lock_entered: bool = False


def is_no_fire_detected(snapshot: HudSnapshot, ir_mode: str) -> bool:
    """Return whether live IR evidence is clearly below detection level.

    Both renderers share this predicate so a low reading silences directional
    guidance as well as changing the Mac status wording.  A credible peak in
    the active sweep keeps guidance available while the wearer turns back
    toward a signal that was actually observed.
    """

    return (
        ir_mode == IR_MODE_SCAN
        and snapshot.ir_raw is not None
        and snapshot.ir_strength is not None
        and not snapshot.guided_scan.signal_confirmed
    )


def classify_snapshot_ir_target(snapshot: HudSnapshot) -> str:
    """Classify the live fire state after debouncing strong sensor readings."""

    mode = classify_ir_target(
        fresh=snapshot.fresh,
        calibrated=snapshot.ir_calibrated,
        raw=snapshot.ir_raw,
        strength=snapshot.ir_strength,
    )
    if mode in (IR_MODE_UNAVAILABLE, IR_MODE_CALIBRATION_NEEDED):
        return mode
    return IR_MODE_LOCK if snapshot.fire_confirmed else IR_MODE_SCAN


class GuidedScanController:
    """Noise-resistant hill-climb guidance for a single forward IR sensor."""

    def __init__(self) -> None:
        self._state = "unavailable"
        self._instruction = "none"
        self._direction = "left"
        self._ema: Optional[float] = None
        self._peak: Optional[float] = None
        self._signal_samples = 0
        self._low_signal_samples = 0
        self._signal_confirmed = False
        self._drop_samples = 0
        self._phase_started = 0.0
        self._last_progress = 0.0
        self._guidance = GuidedScanGuidance(
            state="unavailable",
            instruction="none",
            strength=None,
            peak_strength=None,
            signal_confirmed=False,
        )

    @property
    def guidance(self) -> GuidedScanGuidance:
        return self._guidance

    def reset(self, now: Optional[float] = None) -> GuidedScanGuidance:
        timestamp = time.monotonic() if now is None else float(now)
        self._state = "unavailable"
        self._instruction = "none"
        self._direction = "left"
        self._ema = None
        self._peak = None
        self._signal_samples = 0
        self._low_signal_samples = 0
        self._signal_confirmed = False
        self._drop_samples = 0
        self._phase_started = timestamp
        self._last_progress = timestamp
        self._guidance = GuidedScanGuidance(
            state="unavailable",
            instruction="none",
            strength=None,
            peak_strength=None,
            signal_confirmed=False,
        )
        return self._guidance

    def _start_search(self, strength: float, now: float, *, direction: str) -> None:
        self._state = "search"
        self._direction = direction
        self._instruction = direction
        self._ema = strength
        self._peak = strength
        self._drop_samples = 0
        self._phase_started = now
        self._last_progress = now

    def _reverse(self, now: float) -> None:
        self._direction = "right" if self._direction == "left" else "left"
        self._instruction = self._direction
        self._peak = self._ema
        self._drop_samples = 0
        self._phase_started = now
        self._last_progress = now

    def update(
        self,
        strength: Optional[float],
        *,
        fresh: bool,
        calibrated: bool,
        now: Optional[float] = None,
    ) -> GuidedScanGuidance:
        timestamp = time.monotonic() if now is None else float(now)
        if (
            not fresh
            or not calibrated
            or strength is None
            or not math.isfinite(strength)
        ):
            return self.reset(timestamp)
        value = max(0.0, min(IR_ADC_MAX, float(strength)))

        # Two consecutive averaged samples must clear the onset threshold
        # before either display may show turn guidance. Keep that evidence
        # through a short loss so the wearer can turn back toward the observed
        # signal, then release it after sustained low readings instead of
        # leaving idle arrows latched forever.
        if value >= GUIDED_SCAN_MIN_PEAK:
            self._low_signal_samples = 0
            self._signal_samples = min(
                GUIDED_SCAN_CONFIRM_SAMPLES,
                self._signal_samples + 1,
            )
            if self._signal_samples >= GUIDED_SCAN_CONFIRM_SAMPLES:
                self._signal_confirmed = True
        elif self._signal_confirmed:
            self._low_signal_samples += 1
            if self._low_signal_samples >= GUIDED_SCAN_RELEASE_SAMPLES:
                self._signal_samples = 0
                self._low_signal_samples = 0
                self._signal_confirmed = False
        else:
            self._signal_samples = 0
            self._low_signal_samples = 0

        if self._state == "unavailable":
            self._start_search(value, timestamp, direction="left")
        else:
            assert self._ema is not None
            self._ema += GUIDED_SCAN_EMA_ALPHA * (value - self._ema)

        lock_entered = False
        assert self._ema is not None
        if self._state == "lock":
            if self._ema < IR_LOCK_RELEASE:
                self._start_search(self._ema, timestamp, direction="left")
            else:
                self._instruction = "ahead"
        elif self._ema >= FLAME_HOT:
            self._state = "lock"
            self._instruction = "ahead"
            self._peak = max(self._peak or self._ema, self._ema)
            self._drop_samples = 0
            lock_entered = True
        else:
            previous_peak = self._peak if self._peak is not None else self._ema
            if self._ema > previous_peak:
                self._peak = self._ema
                if self._ema - previous_peak >= 30.0:
                    self._last_progress = timestamp

            peak = self._peak if self._peak is not None else self._ema
            required_drop = max(
                GUIDED_SCAN_DROP_COUNTS,
                peak * GUIDED_SCAN_DROP_RATIO,
            )
            if peak >= GUIDED_SCAN_MIN_PEAK and peak - self._ema >= required_drop:
                self._drop_samples += 1
            else:
                self._drop_samples = 0

            if self._drop_samples >= GUIDED_SCAN_DROP_SAMPLES:
                self._reverse(timestamp)
            elif timestamp - self._last_progress >= GUIDED_SCAN_ALTERNATE_S:
                self._reverse(timestamp)

        self._guidance = GuidedScanGuidance(
            state=self._state,
            instruction=self._instruction,
            strength=value,
            peak_strength=self._peak,
            signal_confirmed=self._signal_confirmed,
            measured_bearing=False,
            lock_entered=lock_entered,
        )
        return self._guidance


@dataclass(frozen=True)
class IrTargetVector:
    """Heading-tagged direction to the strongest calibrated IR sample."""

    bearing: float
    relative: float
    direction: str
    turn_degrees: int
    strength: float


def find_ir_target_vector(
    heading: Optional[float],
    slices: Sequence[float],
    current_strength: Optional[float],
) -> Optional[IrTargetVector]:
    """Return truthful turn guidance after a heading-tagged IR scan.

    A single receiver can remember the strongest direction only when each
    strength sample is paired with a fresh heading. ``current_strength`` is
    used as freshness evidence; it does not need to remain high while the
    wearer turns away from the stored peak.
    """

    if heading is None or current_strength is None or len(slices) != NUM_SLICES:
        return None
    hottest_index = max(range(NUM_SLICES), key=slices.__getitem__)
    hottest_strength = float(slices[hottest_index])
    if not math.isfinite(hottest_strength) or hottest_strength < FLAME_HOT:
        return None
    bearing = (hottest_index * 10.0 + 5.0) % 360.0
    relative = wrap_angle(bearing - heading)
    if abs(relative) <= FACING_ANGLE:
        direction = "ahead"
    elif relative < 0:
        direction = "left"
    else:
        direction = "right"
    return IrTargetVector(
        bearing=bearing,
        relative=relative,
        direction=direction,
        turn_degrees=round(abs(relative)),
        strength=hottest_strength,
    )


@dataclass(frozen=True)
class OledLinkSnapshot:
    ready: bool
    fresh: bool
    age_s: Optional[float]
    accepted_frames: Optional[int]
    displayed_frames: Optional[int]
    crc_errors: Optional[int]
    length_errors: Optional[int]
    packet_timeouts: Optional[int]
    last_sequence: Optional[int]


class VisorHud:
    """Telemetry collection plus RGB and SSD1306 HUD rendering."""

    def __init__(
        self,
        *,
        udp_port: int = 8892,
        sensor_port: Optional[str] = "auto",
        sensor_baud: int = 115200,
        display_serial_device: str = "/dev/nathacks-oled",
        fake: bool = False,
        ir_calibrated: bool = False,
        ir_ambient_raw: Optional[float] = None,
        ir_target_raw: Optional[float] = None,
        target_db_path: Optional[str] = None,
        target_log_cooldown_s: float = TARGET_LOG_COOLDOWN_S,
    ) -> None:
        self.udp_port = udp_port
        self.sensor_port = sensor_port
        self.sensor_baud = sensor_baud
        self.display_serial_device = os.path.realpath(display_serial_device)
        self.fake = fake
        if target_log_cooldown_s < 0 or not math.isfinite(target_log_cooldown_s):
            raise ValueError("target log cooldown must be a finite non-negative value")
        self.target_log_cooldown_s = float(target_log_cooldown_s)
        self.target_history: Optional[TargetHistory] = None
        self._last_target: Optional[TargetSighting] = None
        if target_db_path:
            try:
                self.target_history = TargetHistory(target_db_path)
                self._last_target = self.target_history.latest()
            except (OSError, sqlite3.Error) as error:
                LOG.error("target history unavailable at %s: %s", target_db_path, error)
        if (ir_ambient_raw is None) != (ir_target_raw is None):
            raise ValueError("IR ambient and target calibration values must be supplied together")
        if ir_ambient_raw is not None and ir_target_raw is not None:
            # Validate both range and minimum useful response. The normalized
            # result itself is not retained from this probe.
            normalize_ir_strength(ir_ambient_raw, ir_ambient_raw, ir_target_raw)
            self.ir_ambient_raw = float(ir_ambient_raw)
            self.ir_target_raw = float(ir_target_raw)
        elif fake:
            self.ir_ambient_raw = 0.0
            self.ir_target_raw = IR_ADC_MAX
        else:
            self.ir_ambient_raw = None
            self.ir_target_raw = None
        # ``ir_calibrated=True`` remains a compatibility path for explicitly
        # configured active-high demos. Real hardware should use endpoints so
        # polarity is inferred rather than guessed.
        self._legacy_ir_active_high = bool(
            ir_calibrated and self.ir_ambient_raw is None and not fake
        )
        self.ir_calibrated = bool(
            fake or self._legacy_ir_active_high or self.ir_ambient_raw is not None
        )
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.state: dict[str, Optional[float]] = {
            "heading": None,
            "air": None,
            "flame": 0.0,
            "gas": None,
        }
        self.field_updated = {name: 0.0 for name in self.state}
        self.slices = [-1.0] * NUM_SLICES
        self.gas_baseline: Optional[float] = 1100.0 if fake else None
        self.gas_started: Optional[float] = None
        self.last_update = 0.0
        self.last_fade = time.monotonic()
        self.guided_scan = GuidedScanController()
        self._guided_guidance = self.guided_scan.guidance
        self._target_lock_active = False
        self._fire_confirm_samples = 0
        self._last_target_logged = -math.inf
        # Session-relative yaw is reset every time the sensor ESP restarts, so
        # this visual memory must never be restored from the persistent DB.
        self._session_fire_bearing: Optional[float] = None
        self._session_fire_strength: Optional[float] = None
        self._line_buffer = bytearray()
        self._threads: list[threading.Thread] = []
        self._oled_ready = False
        self._oled_status_updated = 0.0
        self._oled_counters: dict[str, Optional[int]] = {
            "accepted_frames": None,
            "displayed_frames": None,
            "crc_errors": None,
            "length_errors": None,
            "packet_timeouts": None,
            "last_sequence": None,
        }

    def _normalize_ir(self, raw: Optional[float]) -> Optional[float]:
        if raw is None:
            return None
        if self.ir_ambient_raw is not None and self.ir_target_raw is not None:
            return normalize_ir_strength(raw, self.ir_ambient_raw, self.ir_target_raw)
        if self._legacy_ir_active_high:
            return max(0.0, min(IR_ADC_MAX, float(raw)))
        return None

    @staticmethod
    def _apply_thermal_look(image) -> bool:
        """Apply a polished false-colour camera palette before HUD drawing.

        OpenCV runs this full-frame operation in compiled code on the Pi.  The
        renderer remains usable without the optional module, which keeps local
        tests and recovery environments functional.
        """

        if (
            cv2 is None
            or not hasattr(image, "dtype")
            or not hasattr(image, "shape")
            or len(image.shape) < 3
        ):
            return False
        try:
            grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Thermal cameras commonly use automatic gain control. Histogram
            # equalisation gives the visible-light stand-in a similarly rich
            # range while preserving edges and small scene details.
            balanced = cv2.equalizeHist(grey)
            thermal = cv2.applyColorMap(balanced, cv2.COLORMAP_INFERNO)
            image[:] = thermal
        except (AttributeError, TypeError, ValueError):
            LOG.exception("thermal camera look failed; retaining normal colour")
            return False
        return True

    def start(self) -> None:
        udp = threading.Thread(target=self._udp_reader, name="hud-udp", daemon=True)
        udp.start()
        self._threads.append(udp)

        if self.fake:
            source = threading.Thread(target=self._fake_reader, name="hud-demo", daemon=True)
            source.start()
            self._threads.append(source)
        elif self.sensor_port:
            source = threading.Thread(
                target=self._serial_reader, name="hud-serial", daemon=True
            )
            source.start()
            self._threads.append(source)

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self._threads:
            thread.join(timeout=1.5)

    def feed_bytes(self, data: bytes) -> None:
        """Accept text returned on the full-duplex OLED serial connection."""

        if not data:
            return
        self._line_buffer.extend(data)
        if len(self._line_buffer) > 8192:
            del self._line_buffer[:-4096]
        while b"\n" in self._line_buffer:
            raw, _, remainder = self._line_buffer.partition(b"\n")
            self._line_buffer = bytearray(remainder)
            self.feed_line(raw.decode("utf-8", "ignore").strip())

    def feed_line(self, line: str) -> None:
        status = parse_oled_status(line)
        ready = line.strip().upper().startswith("READY VSR1")
        if ready or status:
            now = time.monotonic()
            with self.lock:
                self._oled_ready = True
                self._oled_status_updated = now
                self._oled_counters.update(status)
        values = parse_line(line)
        if values:
            self._handle_values(values)

    def oled_link_snapshot(self) -> OledLinkSnapshot:
        """Return receiver evidence collected on the already-open USB link."""

        now = time.monotonic()
        with self.lock:
            age = (
                max(0.0, now - self._oled_status_updated)
                if self._oled_status_updated > 0
                else None
            )
            return OledLinkSnapshot(
                ready=self._oled_ready,
                fresh=age is not None and age <= OLED_LINK_FRESH_S,
                age_s=age,
                **self._oled_counters,
            )

    def _handle_values(self, values: dict[str, float]) -> None:
        now = time.monotonic()
        sighting: Optional[TargetSighting] = None
        with self.lock:
            for source, target in (
                ("HEADING", "heading"),
                ("AIR", "air"),
                ("FLAME", "flame"),
                ("GAS", "gas"),
            ):
                if source in values:
                    self.state[target] = values[source]
                    self.field_updated[target] = now

            heading = self.state["heading"]
            ir_raw = self.state["flame"]
            ir_strength = self._normalize_ir(ir_raw)
            if "FLAME" in values:
                self._guided_guidance = self.guided_scan.update(
                    ir_strength,
                    fresh=True,
                    calibrated=self.ir_calibrated,
                    now=now,
                )
            # A GPIO10 IR sample may arrive independently from a compass.
            # Never attach it to a bearing that stopped updating earlier.
            if (
                "FLAME" in values
                and ir_strength is not None
                and heading is not None
                and now - self.field_updated["heading"] <= DIRECTION_PAIRING_S
            ):
                index = int(heading // 10) % NUM_SLICES
                # Moderate readings still feed the blue-to-red scan strip.
                # Strong readings are admitted only after the 300 ms fire
                # confirmation below, so one spike cannot create a measured
                # fire bearing.
                if ir_strength < FLAME_HOT:
                    self.slices[index] = max(self.slices[index], ir_strength)

            if "FLAME" in values and ir_strength is not None:
                was_locked = self._target_lock_active
                if was_locked:
                    if ir_strength <= IR_LOCK_RELEASE:
                        self._target_lock_active = False
                        self._fire_confirm_samples = 0
                elif ir_strength >= FLAME_HOT:
                    self._fire_confirm_samples = min(
                        FIRE_CONFIRM_SAMPLES,
                        self._fire_confirm_samples + 1,
                    )
                    if self._fire_confirm_samples >= FIRE_CONFIRM_SAMPLES:
                        self._target_lock_active = True
                else:
                    self._fire_confirm_samples = 0

                locked_now = self._target_lock_active
                paired_heading = (
                    float(heading) % 360.0
                    if heading is not None
                    and now - self.field_updated["heading"] <= DIRECTION_PAIRING_S
                    else None
                )
                if locked_now and paired_heading is not None:
                    index = int(paired_heading // 10) % NUM_SLICES
                    self.slices[index] = max(self.slices[index], ir_strength)
                if locked_now and paired_heading is not None:
                    # A newly acquired source replaces the previous session
                    # marker. During the same lock, keep refining toward the
                    # strongest observed heading.
                    if (
                        not was_locked
                        or self._session_fire_strength is None
                        or ir_strength >= self._session_fire_strength
                    ):
                        self._session_fire_bearing = (
                            int(paired_heading // 10) * 10.0 + 5.0
                        ) % 360.0
                        self._session_fire_strength = float(ir_strength)
                if locked_now and not was_locked:
                    if now - self._last_target_logged >= self.target_log_cooldown_s:
                        bearing = (
                            (int(paired_heading // 10) * 10.0 + 5.0) % 360.0
                            if paired_heading is not None
                            else None
                        )
                        sighting = TargetSighting(
                            seen_at_utc=datetime.now(timezone.utc)
                            .isoformat(timespec="milliseconds")
                            .replace("+00:00", "Z"),
                            ir_raw=float(ir_raw),
                            ir_strength=float(ir_strength),
                            heading=paired_heading,
                            bearing=bearing,
                        )
                        self._last_target = sighting
                        self._last_target_logged = now

            if self.state["gas"] is not None and self.gas_started is None:
                self.gas_started = now
            if (
                self.gas_baseline is None
                and self.gas_started is not None
                and now - self.gas_started >= GAS_WARMUP_S
            ):
                self.gas_baseline = self.state["gas"]
            self.last_update = now

        if sighting is not None and self.target_history is not None:
            try:
                self.target_history.record(sighting)
            except (OSError, sqlite3.Error, ValueError) as error:
                LOG.error("could not save target sighting: %s", error)

    def snapshot(self) -> HudSnapshot:
        now = time.monotonic()
        with self.lock:
            elapsed = max(0.0, now - self.last_fade)
            fade = FADE_PER_30_FPS_FRAME ** (elapsed * 30.0)
            for index, value in enumerate(self.slices):
                if value > 0:
                    self.slices[index] = value * fade
            self.last_fade = now
            field_is_fresh = {
                name: now - updated <= SENSOR_FRESH_S
                for name, updated in self.field_updated.items()
            }
            ir_raw = (
                float(self.state["flame"] or 0.0)
                if field_is_fresh["flame"]
                else None
            )
            if not field_is_fresh["flame"]:
                self._guided_guidance = self.guided_scan.reset(now)
                self._target_lock_active = False
                self._fire_confirm_samples = 0
            return HudSnapshot(
                heading=self.state["heading"] if field_is_fresh["heading"] else None,
                air=self.state["air"] if field_is_fresh["air"] else None,
                flame=ir_raw,
                gas=self.state["gas"] if field_is_fresh["gas"] else None,
                gas_baseline=self.gas_baseline,
                slices=tuple(self.slices),
                fresh=any(field_is_fresh.values()),
                demo=self.fake,
                ir_calibrated=self.ir_calibrated,
                ir_raw=ir_raw,
                ir_strength=self._normalize_ir(ir_raw),
                fire_confirmed=self._target_lock_active,
                last_target=self._last_target,
                session_fire_bearing=self._session_fire_bearing,
                session_fire_strength=self._session_fire_strength,
                guided_scan=self._guided_guidance,
            )

    def _udp_reader(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("0.0.0.0", self.udp_port))
        sock.settimeout(1.0)
        try:
            while not self.stop_event.is_set():
                try:
                    data, _peer = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                for line in data.decode("utf-8", "ignore").splitlines():
                    self.feed_line(line.strip())
        finally:
            sock.close()

    def _serial_candidates(self) -> list[str]:
        if self.sensor_port != "auto":
            return [self.sensor_port] if self.sensor_port else []
        paths = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        return [path for path in paths if os.path.realpath(path) != self.display_serial_device]

    def _serial_reader(self) -> None:
        import serial

        while not self.stop_event.is_set():
            candidates = self._serial_candidates()
            if not candidates:
                self.stop_event.wait(2.0)
                continue
            for port in candidates:
                if self.stop_event.is_set():
                    return
                try:
                    with serial.Serial(port, self.sensor_baud, timeout=1) as stream:
                        while not self.stop_event.is_set():
                            line = stream.readline().decode("utf-8", "ignore").strip()
                            self.feed_line(line)
                except (OSError, serial.SerialException):
                    continue
            self.stop_event.wait(2.0)

    def _fake_reader(self) -> None:
        heading = 0.0
        while not self.stop_event.is_set():
            heading = (heading + 3.0) % 360.0
            difference = abs(wrap_angle(heading - 135.0))
            flame = (
                4000.0 * max(0.0, math.cos(math.radians(difference) * 1.5))
                if difference < 60.0
                else 50.0
            )
            self._handle_values(
                {"HEADING": heading, "AIR": 23.4, "FLAME": flame, "GAS": 1100.0}
            )
            self.stop_event.wait(0.2)

    @staticmethod
    def _heat_color(value: float) -> tuple[int, int, int]:
        amount = max(0.0, min(1.0, value / 4095.0))
        blue, yellow, red = (221, 138, 55), (39, 159, 239), (74, 75, 226)
        if amount < 0.5:
            first, second, mix = blue, yellow, amount * 2.0
        else:
            first, second, mix = yellow, red, (amount - 0.5) * 2.0
        return tuple(
            int(first[channel] + (second[channel] - first[channel]) * mix)
            for channel in range(3)
        )

    @staticmethod
    def _clip_rect(image, x0: int, y0: int, x1: int, y1: int):
        height, width = image.shape[:2]
        return max(0, x0), max(0, y0), min(width, x1), min(height, y1)

    @classmethod
    def _fill_rect(cls, image, rect, color: Sequence[int]) -> None:
        x0, y0, x1, y1 = cls._clip_rect(image, *rect)
        if x1 > x0 and y1 > y0:
            image[y0:y1, x0:x1] = color

    @classmethod
    def _blend_rect(cls, image, rect, color: Sequence[int], alpha: float) -> None:
        x0, y0, x1, y1 = cls._clip_rect(image, *rect)
        if x1 <= x0 or y1 <= y0:
            return
        region = image[y0:y1, x0:x1]
        amount = int(max(0.0, min(1.0, alpha)) * 255)
        mixed = region.astype("uint16") * (255 - amount)
        # Add each channel explicitly to avoid importing NumPy in this module.
        for channel, value in enumerate(color):
            mixed[..., channel] += int(value) * amount
        region[:] = (mixed // 255).astype("uint8")

    def _blend_ellipse(
        self,
        image,
        center: tuple[int, int],
        radii: tuple[int, int],
        color: Sequence[int],
        alpha: float,
    ) -> None:
        """Blend a dependency-light, banded ellipse into the camera image."""

        center_x, center_y = center
        radius_x, radius_y = radii
        if radius_x <= 0 or radius_y <= 0 or alpha <= 0:
            return
        # A few broad contour bands create the synthetic thermal look without
        # dozens of temporary image blends per layer on the Raspberry Pi.
        band_height = max(
            1,
            math.ceil((radius_y * 2) / THERMAL_ELLIPSE_BANDS),
        )
        top = center_y - radius_y
        bottom = center_y + radius_y
        while top < bottom:
            band_bottom = min(bottom, top + band_height)
            sample_y = (top + band_bottom) / 2.0
            normalized_y = (sample_y - center_y) / radius_y
            half_width = round(
                radius_x * math.sqrt(max(0.0, 1.0 - normalized_y * normalized_y))
            )
            if half_width > 0:
                self._blend_rect(
                    image,
                    (center_x - half_width, top, center_x + half_width, band_bottom),
                    color,
                    alpha,
                )
            top = band_bottom

    def _draw_fire_heat_patch(
        self,
        image,
        center: tuple[int, int],
        strength: float,
        live_weight: float,
    ) -> None:
        """Draw a local thermal-style cue without tinting the whole camera."""

        height, width = image.shape[:2]
        scale_x, scale_y = width / 800.0, height / 480.0
        signal = max(0.0, min(IR_ADC_MAX, float(strength)))
        heat = max(
            0.0,
            min(
                1.0,
                (signal - GUIDED_SCAN_MIN_PEAK)
                / (IR_ADC_MAX - GUIDED_SCAN_MIN_PEAK),
            ),
        )
        memory = max(0.0, min(1.0, float(live_weight)))
        visibility = 0.35 + 0.65 * memory
        radius_x = round((52 + 38 * heat) * scale_x)
        radius_y = round((42 + 30 * heat) * scale_y)
        outer_alpha = (0.08 + 0.10 * heat) * visibility

        self._blend_ellipse(
            image,
            center,
            (radius_x, radius_y),
            self._heat_color(signal),
            outer_alpha,
        )
        self._blend_ellipse(
            image,
            center,
            (round(radius_x * 0.68), round(radius_y * 0.68)),
            self._heat_color(min(IR_ADC_MAX, signal + 900.0)),
            outer_alpha * 1.35,
        )
        self._blend_ellipse(
            image,
            center,
            (round(radius_x * 0.34), round(radius_y * 0.34)),
            self._heat_color(min(IR_ADC_MAX, signal + 1700.0)),
            outer_alpha * 1.7,
        )

    def _draw_remembered_fire_patch(
        self,
        image,
        center: tuple[int, int],
    ) -> None:
        """Keep a light-orange marker at the last session fire bearing."""

        height, width = image.shape[:2]
        scale_x, scale_y = width / 800.0, height / 480.0
        radius_x = round(70 * scale_x)
        radius_y = round(54 * scale_y)
        orange = (100, 190, 255)
        self._blend_ellipse(image, center, (radius_x, radius_y), orange, 0.10)
        self._blend_ellipse(
            image,
            center,
            (round(radius_x * 0.67), round(radius_y * 0.67)),
            orange,
            0.16,
        )
        self._blend_ellipse(
            image,
            center,
            (round(radius_x * 0.34), round(radius_y * 0.34)),
            orange,
            0.22,
        )

    @classmethod
    def _draw_line(
        cls,
        image,
        start: tuple[int, int],
        end: tuple[int, int],
        color: Sequence[int],
        thickness: int = 1,
    ) -> None:
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        radius = max(0, thickness // 2)
        while True:
            cls._fill_rect(
                image,
                (x0 - radius, y0 - radius, x0 + radius + 1, y0 + radius + 1),
                color,
            )
            if x0 == x1 and y0 == y1:
                break
            twice = error * 2
            if twice >= dy:
                error += dy
                x0 += step_x
            if twice <= dx:
                error += dx
                y0 += step_y

    @classmethod
    def _draw_text(
        cls,
        image,
        text: str,
        x: int,
        y: int,
        color: Sequence[int],
        scale: int,
    ) -> None:
        cursor = x
        for character in text.upper():
            rows = FONT_5X7.get(character, FONT_5X7["?"])
            for row, bits in enumerate(rows):
                for column in range(5):
                    if bits & (1 << (4 - column)):
                        cls._fill_rect(
                            image,
                            (
                                cursor + column * scale,
                                y + row * scale,
                                cursor + (column + 1) * scale,
                                y + (row + 1) * scale,
                            ),
                            color,
                        )
            cursor += 6 * scale

    def draw_rgb(self, image) -> None:
        """Draw a colored HUD in-place on an RGB888/BGR memory buffer."""

        snapshot = self.snapshot()
        self._apply_thermal_look(image)
        height, width = image.shape[:2]
        scale_x, scale_y = width / 800.0, height / 480.0
        font_scale = max(2, round(min(scale_x, scale_y) * 2))
        white = (245, 245, 245)
        red = (74, 75, 226)
        amber = (39, 159, 239)
        ir_mode = classify_snapshot_ir_target(snapshot)
        target_vector = find_ir_target_vector(
            snapshot.heading, snapshot.slices, snapshot.ir_strength
        )
        # A negative current detection also suppresses every turn instruction
        # below, so the prototype cannot invent a direction before it has
        # observed a credible fire-simulator signal.
        no_fire_detected = is_no_fire_detected(snapshot, ir_mode)

        def draw_target_brackets() -> None:
            """Mark a forward IR lock without inventing a left/right bearing."""

            center = width // 2
            box_w, box_h = int(220 * scale_x), int(160 * scale_y)
            bx0, by0 = center - box_w // 2, height // 2 - box_h // 2
            bx1, by1 = bx0 + box_w, by0 + box_h
            length = int(35 * min(scale_x, scale_y))
            for px, py, sx, sy in (
                (bx0, by0, 1, 1),
                (bx1, by0, -1, 1),
                (bx0, by1, 1, -1),
                (bx1, by1, -1, -1),
            ):
                self._draw_line(image, (px, py), (px + sx * length, py), red, 8)
                self._draw_line(image, (px, py), (px, py + sy * length), red, 8)

        def draw_edge_arrow(left: bool, color: Sequence[int]) -> None:
            edge = int(30 * scale_x) if left else width - int(30 * scale_x)
            direction = 1 if left else -1
            center_y = height // 2
            size = int(55 * min(scale_x, scale_y))
            points = [
                (edge, center_y),
                (edge + direction * size, center_y - size),
                (edge + direction * size, center_y + size),
                (edge, center_y),
            ]
            for first, second in zip(points, points[1:]):
                self._draw_line(image, first, second, color, 10)

        def guided_search_color() -> Sequence[int]:
            # Red means the sweep has encountered a meaningful signal, but
            # the GUIDED SEARCH label remains explicit: without a heading
            # axis this is still an instruction, not a measured bearing.
            strength = snapshot.guided_scan.strength or 0.0
            peak = snapshot.guided_scan.peak_strength or 0.0
            return red if max(strength, peak) >= GUIDED_SCAN_MIN_PEAK else amber

        def draw_fire_label(
            text: str,
            center: tuple[int, int],
            *,
            below: bool,
            color: Sequence[int],
        ) -> None:
            """Attach a readable label to a live or remembered heat patch."""

            label_scale = max(1, font_scale - 1)
            text_width = len(text) * 6 * label_scale
            text_height = 7 * label_scale
            pad_x = max(5, round(6 * scale_x))
            pad_y = max(3, round(4 * scale_y))
            box_width = text_width + 2 * pad_x
            box_height = text_height + 2 * pad_y
            margin = max(4, round(8 * scale_x))
            label_x = max(
                margin,
                min(width - margin - box_width, center[0] - box_width // 2),
            )
            offset = round((78 if below else 66) * scale_y)
            label_y = center[1] + offset if below else center[1] - offset - box_height
            label_y = max(margin, min(height - margin - box_height, label_y))
            self._blend_rect(
                image,
                (label_x, label_y, label_x + box_width, label_y + box_height),
                (0, 0, 0),
                0.72,
            )
            self._draw_text(
                image,
                text,
                label_x + pad_x,
                label_y + pad_y,
                color,
                label_scale,
            )

        # Only a debounced, confirmed fire gets a live heat patch and FIRE
        # label. Guidance may begin at the lower onset threshold, but it does
        # not promote a reflection or one-frame spike into a fire marker.
        fire_patch_strength: Optional[float] = (
            snapshot.ir_strength
            if snapshot.fire_confirmed and snapshot.ir_strength is not None
            else None
        )
        live_center = (width // 2, round(height * 0.47))

        # Session memory is a stable, light-orange bearing cue. It is separate
        # from the decaying strip and persistent database because relative yaw
        # is valid only for the current IMU session.
        if (
            snapshot.heading is not None
            and snapshot.session_fire_bearing is not None
        ):
            remembered_relative = wrap_angle(
                snapshot.session_fire_bearing - snapshot.heading
            )
            if abs(remembered_relative) <= CAMERA_HORIZONTAL_HALF_FOV:
                horizontal_span = width * 0.36
                remembered_x = round(
                    width / 2
                    + remembered_relative
                    / CAMERA_HORIZONTAL_HALF_FOV
                    * horizontal_span
                )
                # If the wearer is still aimed at the remembered bearing and
                # the signal is currently active, it is a live FIRE—not a
                # previous one.  At another bearing, both markers may coexist.
                same_as_live = (
                    fire_patch_strength is not None
                    and abs(remembered_relative) <= FACING_ANGLE
                )
                if not same_as_live:
                    remembered_center = (remembered_x, round(height * 0.47))
                    self._draw_remembered_fire_patch(image, remembered_center)
                    draw_fire_label(
                        "PREVIOUS FIRE DETECTED HERE",
                        remembered_center,
                        below=False,
                        color=amber,
                    )

        if fire_patch_strength is not None:
            self._draw_fire_heat_patch(
                image,
                live_center,
                fire_patch_strength,
                1.0,
            )
            draw_fire_label(
                "FIRE",
                live_center,
                below=True,
                color=red,
            )

        if ir_mode in (IR_MODE_UNAVAILABLE, IR_MODE_CALIBRATION_NEEDED) or snapshot.heading is None:
            if no_fire_detected:
                banner = "NO FIRE DETECTED"
            elif ir_mode == IR_MODE_UNAVAILABLE:
                banner = "WAITING FOR SENSOR"
                if snapshot.fresh:
                    banner = "WAITING FOR FIRE TARGET"
            elif ir_mode == IR_MODE_CALIBRATION_NEEDED:
                banner = (
                    "FIRE SENSOR CALIBRATION NEEDED"
                    if snapshot.ir_raw is None
                    else f"FIRE RAW {snapshot.ir_raw:.0f} - CALIBRATION NEEDED"
                )
            elif ir_mode == IR_MODE_LOCK:
                # The forward receiver can truthfully confirm that it is aimed
                # into a strong IR field, but one channel cannot invent a
                # left/right bearing. The configured threshold is for this
                # emitter demo and still needs real-world calibration.
                banner = f"FIRE TARGET LOCK - SIG {snapshot.ir_strength:.0f}"
                draw_target_brackets()
            else:
                instruction = snapshot.guided_scan.instruction
                if instruction in ("left", "right"):
                    banner = (
                        f"GUIDED SEARCH - TURN {instruction.upper()} - "
                        f"SIG {snapshot.ir_strength:.0f}"
                    )
                    draw_edge_arrow(instruction == "left", guided_search_color())
                else:
                    banner = f"SCAN FOR FIRE - SIG {snapshot.ir_strength:.0f}"
            banner_width = min(
                width - int(32 * scale_x), len(banner) * 6 * font_scale + 24
            )
            banner_left = (width - banner_width) // 2
            banner_top = int(14 * scale_y)
            banner_height = 7 * font_scale + int(18 * scale_y)
            box = (
                banner_left,
                banner_top,
                banner_left + banner_width,
                banner_top + banner_height,
            )
            self._blend_rect(image, box, (0, 0, 0), 0.6)
            self._draw_text(
                image,
                banner,
                box[0] + 12,
                box[1] + int(6 * scale_y),
                white,
                font_scale,
            )
        else:
            x0, x1 = int(20 * scale_x), int(780 * scale_x)
            y0, y1 = int(14 * scale_y), int(54 * scale_y)
            center = width // 2
            pixels_per_degree = (x1 - x0) / STRIP_SPAN
            self._blend_rect(image, (x0, y0, x1, y1), (0, 0, 0), 0.58)

            for index, value in enumerate(snapshot.slices):
                difference = wrap_angle(index * 10.0 + 5.0 - snapshot.heading)
                if abs(difference) > STRIP_SPAN / 2 + 5:
                    continue
                left = max(x0, int(center + (difference - 5) * pixels_per_degree) + 1)
                right = min(x1, int(center + (difference + 5) * pixels_per_degree) - 1)
                if right > left:
                    color = (100, 100, 100) if value < 0 else self._heat_color(value)
                    self._fill_rect(image, (left, y0 + 8, right, y1 - 8), color)

            for angle in range(0, 360, 30):
                difference = wrap_angle(angle - snapshot.heading)
                if abs(difference) > STRIP_SPAN / 2 - 2:
                    continue
                x = int(center + difference * pixels_per_degree)
                self._draw_line(image, (x, y1), (x, y1 + int(6 * scale_y)), white, 2)
                self._draw_text(
                    image,
                    str(angle),
                    x - 10 * font_scale,
                    y1 + int(8 * scale_y),
                    white,
                    max(2, font_scale - 1),
                )
            self._draw_line(image, (center, y0), (center, y1), white, 4)

            guidance = None
            guidance_color = red
            if no_fire_detected:
                guidance = "NO FIRE DETECTED"
                guidance_color = white
            elif target_vector is not None:
                if target_vector.direction == "ahead":
                    draw_target_brackets()
                    guidance = "FIRE TARGET AHEAD"
                else:
                    left = target_vector.direction == "left"
                    draw_edge_arrow(left, red)
                    guidance = (
                        f"TURN {target_vector.direction.upper()} "
                        f"{target_vector.turn_degrees} DEG"
                    )
            elif ir_mode == IR_MODE_LOCK:
                draw_target_brackets()
                guidance = "FIRE TARGET AHEAD"
            elif snapshot.guided_scan.instruction in ("left", "right"):
                instruction = snapshot.guided_scan.instruction
                guidance_color = guided_search_color()
                draw_edge_arrow(instruction == "left", guidance_color)
                guidance = f"GUIDED SEARCH - TURN {instruction.upper()}"

            if guidance is not None:
                guidance_width = min(
                    width - int(32 * scale_x),
                    len(guidance) * 6 * font_scale + 24,
                )
                guidance_left = (width - guidance_width) // 2
                guidance_top = int(82 * scale_y)
                guidance_height = 7 * font_scale + int(18 * scale_y)
                self._blend_rect(
                    image,
                    (
                        guidance_left,
                        guidance_top,
                        guidance_left + guidance_width,
                        guidance_top + guidance_height,
                    ),
                    (0, 0, 0),
                    0.65,
                )
                self._draw_text(
                    image,
                    guidance,
                    guidance_left + 12,
                    guidance_top + int(6 * scale_y),
                    guidance_color,
                    font_scale,
                )

        # The Mac is the information-rich display. Keep the live camera clear
        # through the centre and reserve four compact cards along the bottom
        # for real sensor detail and explicit unavailable states.
        dim = (160, 160, 160)
        cyan = (235, 221, 101)
        small_scale = max(1, font_scale - 1)

        strength_text = (
            "SIG --"
            if snapshot.ir_strength is None
            else f"SIG {snapshot.ir_strength:.0f}"
        )
        raw_text = "RAW --" if snapshot.ir_raw is None else f"RAW {snapshot.ir_raw:.0f}"
        if ir_mode == IR_MODE_LOCK:
            ir_detail, ir_color = f"LOCK {raw_text}", red
        elif no_fire_detected:
            ir_detail, ir_color = "NO FIRE DETECTED", white
        elif ir_mode == IR_MODE_SCAN:
            ir_detail, ir_color = f"SCAN {raw_text}", cyan
        elif ir_mode == IR_MODE_CALIBRATION_NEEDED:
            ir_detail, ir_color = "CAL NEEDED", amber
        else:
            ir_detail, ir_color = "OFFLINE", dim

        heading_value = (
            "YAW --" if snapshot.heading is None else f"YAW {snapshot.heading:.0f} DEG"
        )
        heading_detail = "NO SENSOR" if snapshot.heading is None else "RELATIVE HEADING"
        air_value = "--" if snapshot.air is None else f"{snapshot.air:.1f} C"
        air_detail = "NO SENSOR" if snapshot.air is None else "REPORTING"

        if snapshot.gas is None:
            gas_value, gas_detail, gas_color = "--", "NO SENSOR", dim
        elif snapshot.gas_baseline is None:
            gas_value, gas_detail, gas_color = f"{snapshot.gas:.0f}", "WARMUP", amber
        elif snapshot.gas > snapshot.gas_baseline + GAS_MARGIN:
            gas_value, gas_detail, gas_color = f"{snapshot.gas:.0f}", "HIGH", red
        else:
            # A raw gas reading is not evidence that the surrounding air is safe.
            gas_value, gas_detail, gas_color = f"{snapshot.gas:.0f}", "REPORTING", white

        cards = (
            ("FIRE TARGET", strength_text, ir_detail, ir_color),
            (
                "HEADING",
                heading_value,
                heading_detail,
                white if snapshot.heading is not None else dim,
            ),
            (
                "AIR TEMP",
                air_value,
                air_detail,
                white if snapshot.air is not None else dim,
            ),
            ("GAS", gas_value, gas_detail, gas_color),
        )
        panel_margin = int(16 * scale_x)
        panel_gap = max(4, int(8 * scale_x))
        panel_y = height - int(86 * scale_y)
        panel_h = int(72 * scale_y)
        panel_w = (width - 2 * panel_margin - 3 * panel_gap) // 4
        for index, (title, value, detail, value_color) in enumerate(cards):
            panel_x = panel_margin + index * (panel_w + panel_gap)
            self._blend_rect(
                image,
                (panel_x, panel_y, panel_x + panel_w, panel_y + panel_h),
                (0, 0, 0),
                0.68,
            )
            self._draw_text(
                image,
                title,
                panel_x + int(8 * scale_x),
                panel_y + int(6 * scale_y),
                dim,
                small_scale,
            )
            self._draw_text(
                image,
                value,
                panel_x + int(8 * scale_x),
                panel_y + int(25 * scale_y),
                value_color,
                font_scale,
            )
            self._draw_text(
                image,
                detail,
                panel_x + int(8 * scale_x),
                panel_y + int(51 * scale_y),
                value_color,
                small_scale,
            )

        # Database history is informational only. A stored relative bearing is
        # never fed back into guidance because gyro-zero may change on restart.
        if snapshot.last_target is not None:
            seen = snapshot.last_target.seen_at_utc
            clock = seen[11:19] if len(seen) >= 19 else seen[:19]
            if snapshot.last_target.has_relative_bearing:
                last_text = (
                    f"LAST FIRE {clock} UTC - REL BEARING "
                    f"{snapshot.last_target.bearing:.0f} DEG"
                )
            else:
                last_text = f"LAST FIRE {clock} UTC - AHEAD - LOCATION UNKNOWN"
            history_y = panel_y - int(22 * scale_y)
            history_w = min(
                width - 2 * panel_margin,
                len(last_text) * 6 * small_scale + int(16 * scale_x),
            )
            history_x = (width - history_w) // 2
            self._blend_rect(
                image,
                (history_x, history_y, history_x + history_w, panel_y - 2),
                (0, 0, 0),
                0.68,
            )
            self._draw_text(
                image,
                last_text,
                history_x + int(8 * scale_x),
                history_y + int(4 * scale_y),
                amber,
                small_scale,
            )
        if snapshot.demo:
            self._draw_text(
                image,
                "DEMO",
                width - 30 * font_scale,
                int(16 * scale_y),
                red,
                font_scale,
            )

    @staticmethod
    def _oled_set(frame: bytearray, x: int, y: int, value: bool = True) -> None:
        if not (0 <= x < 128 and 0 <= y < 64):
            return
        index = (y >> 3) * 128 + x
        mask = 1 << (y & 7)
        if value:
            frame[index] |= mask
        else:
            frame[index] &= ~mask

    @classmethod
    def _oled_line(
        cls,
        frame: bytearray,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        value: bool = True,
    ) -> None:
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx + dy
        while True:
            cls._oled_set(frame, x0, y0, value)
            if x0 == x1 and y0 == y1:
                return
            twice = error * 2
            if twice >= dy:
                error += dy
                x0 += step_x
            if twice <= dx:
                error += dx
                y0 += step_y

    @classmethod
    def _oled_thick_line(
        cls,
        frame: bytearray,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        thickness: int = 3,
        value: bool = True,
    ) -> None:
        radius = max(0, thickness // 2)
        for offset_x in range(-radius, radius + 1):
            for offset_y in range(-radius, radius + 1):
                cls._oled_line(
                    frame,
                    x0 + offset_x,
                    y0 + offset_y,
                    x1 + offset_x,
                    y1 + offset_y,
                    value,
                )

    @classmethod
    def _oled_arrow_path(
        cls, frame: bytearray, points: Sequence[tuple[int, int]]
    ) -> None:
        """Draw one compact camera-visible arrow with a dark separation halo."""

        # Clear a wider path first so the white arrow remains legible over
        # either black or white regions of the one-bit processed camera image.
        for start, end in zip(points, points[1:]):
            cls._oled_thick_line(
                frame, *start, *end, thickness=5, value=False
            )
        for start, end in zip(points, points[1:]):
            cls._oled_thick_line(
                frame, *start, *end, thickness=3, value=True
            )

    @classmethod
    def _oled_direction_arrow(cls, frame: bytearray, left: bool) -> None:
        """Overlay a small edge arrow while retaining the camera as the HUD."""

        if left:
            points = (
                (5, 32),
                (20, 19),
                (20, 26),
                (31, 26),
                (31, 38),
                (20, 38),
                (20, 45),
                (5, 32),
            )
        else:
            points = (
                (122, 32),
                (107, 19),
                (107, 26),
                (96, 26),
                (96, 38),
                (107, 38),
                (107, 45),
                (122, 32),
            )
        cls._oled_arrow_path(frame, points)

    def draw_oled(self, payload: bytes) -> bytes:
        """Keep processed vision dominant and add at most one small turn cue."""

        if len(payload) != 1024:
            raise ValueError("OLED payload must be exactly 1024 bytes")
        snapshot = self.snapshot()
        ir_mode = classify_snapshot_ir_target(snapshot)
        frame = bytearray(payload)
        target_vector = find_ir_target_vector(
            snapshot.heading, snapshot.slices, snapshot.ir_strength
        )
        no_fire_detected = is_no_fire_detected(snapshot, ir_mode)

        # An unavailable or uncalibrated IR channel supplies no trustworthy
        # guidance. A fresh low reading likewise means there is no detected
        # target to point toward, so leave every processed-camera pixel
        # untouched. Once a credible signal exists, choose at most one small
        # left/right cue; the OLED intentionally has no text, large overlay,
        # or ahead/up symbol.
        if (
            ir_mode in (IR_MODE_UNAVAILABLE, IR_MODE_CALIBRATION_NEEDED)
            or no_fire_detected
        ):
            return bytes(frame)

        if target_vector is not None and target_vector.direction in ("left", "right"):
            self._oled_direction_arrow(frame, target_vector.direction == "left")
        elif snapshot.guided_scan.instruction in ("left", "right"):
            self._oled_direction_arrow(
                frame, snapshot.guided_scan.instruction == "left"
            )
        return bytes(frame)
