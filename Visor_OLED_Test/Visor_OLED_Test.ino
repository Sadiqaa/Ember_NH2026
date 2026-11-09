// Freenove ESP32-S3-WROOM USB-UART receiver for 128x64 1-bit camera frames.
// Installed OLED: SSD1306 I2C 0x3D; SDA=GPIO8, SCL=GPIO9.
// VIN -> 3V3, GND -> GND. Leave 3Vo, CS, DC and RST unconnected.
// IR flame sensor: AO -> GPIO10, VCC -> 3V3, GND -> GND.

#ifndef USE_SPI
#define USE_SPI 0
#endif
#ifndef USE_SH1106
#define USE_SH1106 0
#endif
#ifndef MIRROR_TEXT
#define MIRROR_TEXT 0
#endif
#ifndef ENABLE_MPU6050
#define ENABLE_MPU6050 1
#endif
// MQ-2 modules are normally powered from 5 V and can expose close to 5 V on
// AO. Leave the input disabled until a resistor divider has been installed and
// its ESP32 ADC pin is intentionally selected in the build flags, for example:
//   -DMQ2_ADC_PIN=4 -DMQ2_DIVIDER_CONFIRMED=1
#ifndef MQ2_ADC_PIN
#define MQ2_ADC_PIN -1
#endif
#ifndef MQ2_DIVIDER_CONFIRMED
#define MQ2_DIVIDER_CONFIRMED 0
#endif

#if (USE_SPI != 0 && USE_SPI != 1) || \
    (USE_SH1106 != 0 && USE_SH1106 != 1) || \
    (MIRROR_TEXT != 0 && MIRROR_TEXT != 1) || \
    (ENABLE_MPU6050 != 0 && ENABLE_MPU6050 != 1) || \
    (MQ2_DIVIDER_CONFIRMED != 0 && MQ2_DIVIDER_CONFIRMED != 1)
#error "Boolean build settings must be 0 or 1."
#endif
#if MQ2_ADC_PIN >= 0 && MQ2_DIVIDER_CONFIRMED != 1
#error "MQ-2 AO can exceed 3.3 V: install a divider, then set MQ2_DIVIDER_CONFIRMED=1."
#endif

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <cstring>

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This firmware targets the Freenove ESP32-S3-WROOM board."
#endif

// I2C uses the board's conventional GPIO8/GPIO9 pair. The SPI assignments are
// kept for the optional build profiles and use exposed, adjacent header pins.
constexpr uint8_t OLED_CLOCK = 12;
constexpr uint8_t OLED_DATA = 11;
constexpr uint8_t OLED_CS = 47;
constexpr uint8_t OLED_DC = 13;
constexpr uint8_t OLED_RESET = 14;
constexpr uint8_t OLED_SDA = 8;
constexpr uint8_t OLED_SCL = 9;
constexpr uint8_t OLED_I2C_ADDRESS = 0x3D;
constexpr uint8_t IR_FLAME_PIN = 10;
constexpr uint8_t MODE_BUTTON = 0;
constexpr int8_t MQ2_PIN = MQ2_ADC_PIN;
// The ESP32-S3 Wire/U8g2 transport can stall while U8g2 reinitializes an
// already-running custom-pin I2C bus. For the installed SSD1306, keep U8g2 as
// the framebuffer/drawing engine but own the small I2C transport directly.
constexpr bool USE_DIRECT_SSD1306_I2C = USE_SPI == 0 && USE_SH1106 == 0;
static_assert(MQ2_PIN == -1 || (MQ2_PIN >= 1 && MQ2_PIN <= 7),
              "Use an unused ADC1 input GPIO1..GPIO7 for the divided MQ-2 AO signal.");

constexpr uint32_t SERIAL_BAUD = 921600;
constexpr size_t SERIAL_RX_BUFFER_BYTES = 4096;
constexpr uint32_t I2C_CLOCK_HZ = 400000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint16_t FRAME_WIDTH = 128;
constexpr uint16_t FRAME_HEIGHT = 64;
constexpr uint16_t FRAME_BYTES = FRAME_WIDTH * FRAME_HEIGHT / 8;
constexpr uint32_t PACKET_TIMEOUT_MS = 250;
constexpr uint32_t STREAM_TIMEOUT_MS = 1500;
constexpr uint32_t ANIMATION_INTERVAL_MS = 50;
constexpr uint32_t SENSOR_INTERVAL_MS = 100;
constexpr uint32_t STATUS_INTERVAL_MS = 2000;
constexpr uint8_t SENSOR_AVERAGE_SAMPLES = 4;
constexpr uint32_t MPU_SAMPLE_INTERVAL_US = 10000;
constexpr uint32_t MPU_HEADING_FRESH_US = 250000;
constexpr uint16_t MPU_CALIBRATION_SAMPLES = 150;
constexpr uint8_t MPU_MAX_CONSECUTIVE_FAILURES = 5;
constexpr float MPU_GYRO_SCALE = 131.0F;  // LSB/(degree/second), +/-250 dps.
constexpr uint8_t PACKET_MAGIC[] = {'V', 'S', 'R', '1'};
static_assert(FRAME_BYTES == 1024, "VSR1 requires 1024-byte frames.");

constexpr uint8_t MPU6050_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU6050_ADDRESS_HIGH = 0x69;
constexpr uint8_t MPU6050_REG_SAMPLE_RATE = 0x19;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_GYRO_Z = 0x47;
constexpr uint8_t MPU6050_REG_POWER = 0x6B;
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;

const u8g2_cb_t *const OLED_ROTATION = MIRROR_TEXT ? U8G2_MIRROR : U8G2_R0;

#if USE_SPI
#if USE_SH1106
U8G2_SH1106_128X64_NONAME_F_4W_SW_SPI display(
    OLED_ROTATION, OLED_CLOCK, OLED_DATA, OLED_CS, OLED_DC, OLED_RESET);
#else
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI display(
    OLED_ROTATION, OLED_CLOCK, OLED_DATA, OLED_CS, OLED_DC, OLED_RESET);
#endif
#else
#if USE_SH1106
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(
    OLED_ROTATION, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(
    OLED_ROTATION, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
#endif
#endif

enum class ReceiveState : uint8_t { SEEK_MAGIC, READ_HEADER, READ_PAYLOAD, READ_CRC };

bool displayReady = false;
bool i2cBusReady = false;
bool streamActive = false;
bool framePending = false;
bool fallbackStatusVisible = true;
bool alignmentMode = true;
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
uint32_t lastButtonChangeMs = 0;
uint32_t lastAnimationMs = 0;
uint32_t lastValidFrameMs = 0;
uint32_t lastSerialByteMs = 0;
uint32_t lastSensorMs = 0;
uint32_t lastStatusMs = 0;
uint32_t acceptedFrames = 0;
uint32_t displayedFrames = 0;
uint32_t crcErrors = 0;
uint32_t lengthErrors = 0;
uint32_t packetTimeouts = 0;

bool mpuDetected = false;
bool mpuHeadingReady = false;
uint8_t mpuAddress = 0;
uint16_t mpuCalibrationCount = 0;
uint16_t mpuReadFailures = 0;
int64_t mpuGyroZAccumulator = 0;
float mpuGyroZBias = 0.0F;
float relativeHeadingDegrees = 0.0F;
uint32_t lastMpuSampleUs = 0;
uint32_t lastMpuSuccessUs = 0;

ReceiveState receiveState = ReceiveState::SEEK_MAGIC;
uint8_t magicIndex = 0;
uint8_t header[4] = {};
uint8_t headerIndex = 0;
uint8_t crcBytes[2] = {};
uint8_t crcIndex = 0;
uint8_t receiveBuffer[FRAME_BYTES] = {};
uint16_t payloadIndex = 0;
uint16_t packetSequence = 0;
uint16_t pendingSequence = 0;
uint16_t displayedSequence = 0;
uint16_t runningCrc = 0xFFFF;

struct Star { uint8_t x, y, speed; };
Star stars[] = {
    {4, 8, 1}, {18, 29, 2}, {32, 14, 1}, {47, 50, 3}, {61, 22, 2},
    {76, 6, 1}, {89, 39, 3}, {104, 18, 2}, {117, 54, 1}, {125, 31, 3}};
uint16_t animationFrame = 0;

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false, xorout=0.
// The standard "123456789" test vector produces 0x29B1.
uint16_t crc16Update(uint16_t crc, uint8_t value) {
  crc ^= static_cast<uint16_t>(value) << 8;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                         : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

void resetReceiver() {
  receiveState = ReceiveState::SEEK_MAGIC;
  magicIndex = 0;
  headerIndex = 0;
  payloadIndex = 0;
  crcIndex = 0;
  runningCrc = 0xFFFF;
}

bool writeSsd1306I2c(uint8_t control, const uint8_t *bytes, size_t length) {
  if (!USE_DIRECT_SSD1306_I2C) return false;
  Wire.beginTransmission(OLED_I2C_ADDRESS);
  bool buffered = Wire.write(control) == 1;
  if (length > 0) buffered = buffered && Wire.write(bytes, length) == length;
  // Every transaction ends with STOP. This keeps the shared bus recoverable
  // and lets the ESP32 Wire timeout bound a disconnected/stuck display.
  const uint8_t status = Wire.endTransmission(true);
  return buffered && status == 0;
}

bool initializeDisplayTransport() {
  if (USE_DIRECT_SSD1306_I2C) {
  // SSD1306 128x64, internal charge pump, horizontal addressing. These are
  // controller commands only; U8g2 still owns all drawing and the 1 KiB buffer.
  static const uint8_t initCommands[] = {
      0xAE,        // display off
      0xD5, 0x80,  // clock divide / oscillator
      0xA8, 0x3F,  // multiplex ratio 1/64
      0xD3, 0x00,  // display offset
      0x40,        // display start line 0
      0x8D, 0x14,  // charge pump on
      0x20, 0x00,  // horizontal addressing mode
      0xA1,        // segment remap
      0xC8,        // COM scan direction
      0xDA, 0x12,  // COM pin configuration for 128x64
      0x81, 0xFF,  // maximum contrast
      0xD9, 0xF1,  // pre-charge period
      0xDB, 0x40,  // VCOMH deselect level
      0xA4,        // resume RAM display
      0xA6,        // normal (not inverted)
      0x2E,        // deactivate scroll
      0xAF,        // display on
  };
    return writeSsd1306I2c(0x00, initCommands, sizeof(initCommands));
  }
  display.begin();
  return true;
}

bool flushDisplayBuffer() {
  if (USE_DIRECT_SSD1306_I2C) {
    static const uint8_t addressWindow[] = {
        0x21, 0x00, 0x7F,  // columns 0..127
        0x22, 0x00, 0x07,  // pages 0..7
    };
    if (!writeSsd1306I2c(0x00, addressWindow, sizeof(addressWindow))) return false;

    constexpr size_t CHUNK_BYTES = 32;
    const uint8_t *const buffer = display.getBufferPtr();
    for (size_t offset = 0; offset < FRAME_BYTES; offset += CHUNK_BYTES) {
      const size_t count = min(CHUNK_BYTES, FRAME_BYTES - offset);
      if (!writeSsd1306I2c(0x40, buffer + offset, count)) return false;
    }
    return true;
  }
  display.sendBuffer();
  return true;
}

bool sendDisplayBuffer() {
  if (flushDisplayBuffer()) return true;
  Serial.println("OLED: frame transfer failed; display disabled until reset.");
  displayReady = false;
  return false;
}

void drawAlignmentTarget() {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  display.drawHLine(37, 32, 55);
  display.drawVLine(64, 10, 45);
  display.drawCircle(64, 32, 8);
  display.drawCircle(64, 32, 16);
  display.drawDisc(64, 32, 2);
  display.drawBox(3, 3, 5, 5);
  display.drawBox(120, 3, 5, 5);
  display.drawBox(3, 56, 5, 5);
  display.drawBox(120, 56, 5, 5);
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(46, 8, "FOCUS");
  display.drawStr(48, 63, "HUD");
  sendDisplayBuffer();
}

void drawAnimationFrame() {
  display.clearBuffer();
  for (Star &star : stars) {
    display.drawPixel(star.x, star.y);
    if (star.speed > 1) display.drawPixel((star.x + 1) & 127, star.y);
    star.x = (star.x + star.speed) & 127;
  }
  display.drawDisc(108, 13, 8);
  display.setDrawColor(0);
  display.drawDisc(104, 10, 7);
  display.setDrawColor(1);
  const int8_t bob = (animationFrame / 5) % 6;
  const int16_t shipY = 31 + (bob < 3 ? bob : 5 - bob);
  display.drawTriangle(50, shipY, 68, shipY + 6, 50, shipY + 12);
  display.drawBox(42, shipY + 3, 12, 6);
  display.drawLine(42, shipY + 4, 35 - (animationFrame & 3), shipY + 4);
  display.drawLine(42, shipY + 7, 37 - (animationFrame & 1), shipY + 7);
  display.drawHLine(0, 58, 128);
  for (uint8_t x = animationFrame & 7; x < 128; x += 8) {
    display.drawLine(x, 59, x + 4, 63);
  }
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(2, 8, "ESP32 SPACE LOOP");
  sendDisplayBuffer();
  ++animationFrame;
}

void drawStreamStatus(bool signalLost) {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(26, 14, "PI CAMERA");
  display.drawHLine(12, 20, 104);
  display.setFont(u8g2_font_5x7_tr);
  display.drawStr(signalLost ? 34 : 38, 34,
                  signalLost ? "SIGNAL LOST" : "WAITING...");
  display.drawStr(36, 47, "USB-UART");
  display.drawStr(28, 59, "BOOT: TEST");
  sendDisplayBuffer();
  fallbackStatusVisible = true;
}

void setFallbackMode(bool useAlignment) {
  alignmentMode = useAlignment;
  fallbackStatusVisible = false;
  if (alignmentMode) {
    drawAlignmentTarget();
    Serial.println("Fallback: mirrored optical alignment target");
  } else {
    drawAnimationFrame();
    Serial.println("Fallback: mirrored space animation");
  }
}

// Incoming payloads are logical, unmirrored images. U8g2's mirror callback
// transforms drawing calls, but not raw bytes copied into its buffer, so mirror
// streamed frames explicitly to match the optical fallback screens.
void stageValidatedFrame(uint32_t now) {
  uint8_t *const destination = display.getBufferPtr();
#if MIRROR_TEXT
  for (uint8_t page = 0; page < FRAME_HEIGHT / 8; ++page) {
    const uint16_t pageStart = static_cast<uint16_t>(page) * FRAME_WIDTH;
    for (uint16_t x = 0; x < FRAME_WIDTH; ++x) {
      destination[pageStart + FRAME_WIDTH - 1 - x] = receiveBuffer[pageStart + x];
    }
  }
#else
  memcpy(destination, receiveBuffer, FRAME_BYTES);
#endif
  framePending = true;
  pendingSequence = packetSequence;
  streamActive = true;
  fallbackStatusVisible = false;
  // Use the loop's timestamp. Taking a newer millis() value here while the
  // caller later compares against its older timestamp can underflow the
  // unsigned timeout calculation and immediately report SIGNAL LOST.
  lastValidFrameMs = now;
  ++acceptedFrames;
}

void consumeSerialByte(uint8_t value, uint32_t now) {
  lastSerialByteMs = now;
  switch (receiveState) {
    case ReceiveState::SEEK_MAGIC:
      if (value == PACKET_MAGIC[magicIndex]) {
        if (++magicIndex == sizeof(PACKET_MAGIC)) {
          receiveState = ReceiveState::READ_HEADER;
          headerIndex = 0;
          runningCrc = 0xFFFF;
          magicIndex = 0;
        }
      } else {
        // VSR1 has no proper prefix that is also a suffix; a fresh V can start
        // the next candidate immediately.
        magicIndex = value == PACKET_MAGIC[0] ? 1 : 0;
      }
      break;

    case ReceiveState::READ_HEADER:
      header[headerIndex++] = value;
      runningCrc = crc16Update(runningCrc, value);
      if (headerIndex == sizeof(header)) {
        packetSequence = static_cast<uint16_t>(header[0]) |
                         (static_cast<uint16_t>(header[1]) << 8);
        const uint16_t payloadLength = static_cast<uint16_t>(header[2]) |
                                       (static_cast<uint16_t>(header[3]) << 8);
        if (payloadLength != FRAME_BYTES) {
          ++lengthErrors;
          resetReceiver();
        } else {
          payloadIndex = 0;
          receiveState = ReceiveState::READ_PAYLOAD;
        }
      }
      break;

    case ReceiveState::READ_PAYLOAD:
      receiveBuffer[payloadIndex++] = value;
      runningCrc = crc16Update(runningCrc, value);
      if (payloadIndex == FRAME_BYTES) {
        crcIndex = 0;
        receiveState = ReceiveState::READ_CRC;
      }
      break;

    case ReceiveState::READ_CRC:
      crcBytes[crcIndex++] = value;
      if (crcIndex == sizeof(crcBytes)) {
        const uint16_t receivedCrc = static_cast<uint16_t>(crcBytes[0]) |
                                     (static_cast<uint16_t>(crcBytes[1]) << 8);
        if (receivedCrc == runningCrc) {
          stageValidatedFrame(now);
        } else {
          ++crcErrors;
        }
        resetReceiver();
      }
      break;
  }
}

void receiveSerialFrames(uint32_t now) {
  if (receiveState != ReceiveState::SEEK_MAGIC &&
      now - lastSerialByteMs > PACKET_TIMEOUT_MS) {
    ++packetTimeouts;
    resetReceiver();
  }

  // Drain only bytes that were already queued. If multiple complete packets
  // accumulated while I2C was busy, send only the newest one to the OLED.
  size_t queuedBytes = Serial.available();
  while (queuedBytes-- > 0) {
    const int incoming = Serial.read();
    if (incoming >= 0) consumeSerialByte(static_cast<uint8_t>(incoming), now);
  }

  if (framePending) {
    const bool frameDisplayed = sendDisplayBuffer();
    framePending = false;
    if (frameDisplayed) {
      displayedSequence = pendingSequence;
      ++displayedFrames;
    }
  }
}

void updateButton(uint32_t now) {
  const bool reading = digitalRead(MODE_BUTTON);
  if (reading != lastButtonReading) {
    lastButtonReading = reading;
    lastButtonChangeMs = now;
  }
  if (now - lastButtonChangeMs >= 30 && reading != stableButtonState) {
    stableButtonState = reading;
    if (stableButtonState == LOW) {
      alignmentMode = !alignmentMode;
      if (!streamActive) setFallbackMode(alignmentMode);
    }
  }
}

bool writeI2cRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  const bool buffered = Wire.write(reg) == 1 && Wire.write(value) == 1;
  // Always finish with STOP, including a local buffer error, so TwoWire never
  // retains its mutex as a pending repeated-start transaction.
  const uint8_t status = Wire.endTransmission(true);
  return buffered && status == 0;
}

bool readI2cRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) {
  if (length == 0) return true;
  Wire.beginTransmission(address);
  const bool buffered = Wire.write(reg) == 1;
  // MPU6050 retains its register pointer across STOP. Avoid endTransmission(false):
  // on ESP32 that deliberately holds Wire's mutex until requestFrom completes.
  const uint8_t pointerStatus = Wire.endTransmission(true);
  if (!buffered || pointerStatus != 0) return false;

  const uint8_t received =
      Wire.requestFrom(address, length, static_cast<uint8_t>(true));
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t index = 0; index < length; ++index) {
    if (!Wire.available()) return false;
    data[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

uint8_t probeI2cAddress(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission(true);
}

bool startI2cBus() {
  if (!Wire.begin(OLED_SDA, OLED_SCL, I2C_CLOCK_HZ)) {
    Serial.printf("I2C: failed to start on SDA=%u SCL=%u.\n", OLED_SDA, OLED_SCL);
    i2cBusReady = false;
    return false;
  }
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  i2cBusReady = true;
  return true;
}

bool recoverI2cBus(const char *reason) {
  Serial.printf("I2C: restarting host after %s.\n", reason);
  Wire.end();
  delay(1);
  const bool recovered = startI2cBus();
  Serial.printf("I2C: restart %s.\n", recovered ? "complete" : "failed");
  return recovered;
}

bool beginMpu6050() {
#if !ENABLE_MPU6050
  Serial.println("MPU6050: disabled by build setting; HEADING unavailable.");
  return false;
#else
  for (const uint8_t address : {MPU6050_ADDRESS_LOW, MPU6050_ADDRESS_HIGH}) {
    // A STOP-delimited address probe prevents a missing optional MPU from ever
    // entering the register-read path on the OLED's shared I2C bus.
    if (probeI2cAddress(address) != 0) continue;

    uint8_t identity = 0;
    if (!readI2cRegisters(address, MPU6050_REG_WHO_AM_I, &identity, 1) ||
        (identity & 0x7E) != 0x68) {
      Serial.printf("MPU6050: address 0x%02X ACKed but WHO_AM_I was invalid.\n",
                    address);
      continue;
    }
    if (!writeI2cRegister(address, MPU6050_REG_POWER, 0x01) ||
        !writeI2cRegister(address, MPU6050_REG_CONFIG, 0x03) ||
        !writeI2cRegister(address, MPU6050_REG_SAMPLE_RATE, 0x09) ||
        !writeI2cRegister(address, MPU6050_REG_GYRO_CONFIG, 0x00)) {
      Serial.printf("MPU6050: configuration failed at 0x%02X.\n", address);
      continue;
    }
    mpuAddress = address;
    mpuDetected = true;
    mpuHeadingReady = false;
    mpuCalibrationCount = 0;
    mpuGyroZAccumulator = 0;
    mpuReadFailures = 0;
    lastMpuSampleUs = micros();
    lastMpuSuccessUs = 0;
    Serial.printf("MPU6050: detected at 0x%02X; keep the visor still for %.1f s gyro calibration.\n",
                  address,
                  MPU_CALIBRATION_SAMPLES * MPU_SAMPLE_INTERVAL_US / 1000000.0F);
    return true;
  }
  Serial.println("MPU6050: not detected at 0x68/0x69; HEADING unavailable.");
  return false;
#endif
}

void updateMpu6050(uint32_t nowUs) {
  if (!mpuDetected) return;
  const uint32_t elapsedUs = nowUs - lastMpuSampleUs;
  if (elapsedUs < MPU_SAMPLE_INTERVAL_US) return;
  lastMpuSampleUs = nowUs;

  uint8_t bytes[2] = {};
  if (!readI2cRegisters(mpuAddress, MPU6050_REG_GYRO_Z, bytes, sizeof(bytes))) {
    ++mpuReadFailures;
    if (mpuReadFailures == 1) {
      Serial.printf("MPU6050: read failure count=%u; HEADING held unavailable if stale.\n",
                    mpuReadFailures);
    }
    if (mpuReadFailures >= MPU_MAX_CONSECUTIVE_FAILURES) {
      Serial.printf("MPU6050: disabling after %u consecutive read failures.\n",
                    mpuReadFailures);
      mpuDetected = false;
      mpuHeadingReady = false;
      recoverI2cBus("MPU6050 read failures");
    }
    return;
  }
  mpuReadFailures = 0;
  lastMpuSuccessUs = nowUs;
  const int16_t gyroZ = static_cast<int16_t>(
      (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1]);

  if (!mpuHeadingReady) {
    mpuGyroZAccumulator += gyroZ;
    if (++mpuCalibrationCount >= MPU_CALIBRATION_SAMPLES) {
      mpuGyroZBias = static_cast<float>(mpuGyroZAccumulator) /
                     static_cast<float>(mpuCalibrationCount);
      relativeHeadingDegrees = 0.0F;
      mpuHeadingReady = true;
      Serial.printf("MPU6050: relative heading ready (gyro-Z bias %.2f LSB).\n",
                    mpuGyroZBias);
    }
    return;
  }

  // A long scheduling gap should not turn into a large fictitious rotation.
  const float deltaSeconds = elapsedUs > 100000 ? 0.0F : elapsedUs / 1000000.0F;
  relativeHeadingDegrees +=
      (static_cast<float>(gyroZ) - mpuGyroZBias) / MPU_GYRO_SCALE * deltaSeconds;
  while (relativeHeadingDegrees >= 360.0F) relativeHeadingDegrees -= 360.0F;
  while (relativeHeadingDegrees < 0.0F) relativeHeadingDegrees += 360.0F;
}

void publishSensorTelemetry(uint32_t now) {
  if (now - lastSensorMs < SENSOR_INTERVAL_MS) return;
  lastSensorMs = now;

  uint32_t total = 0;
  for (uint8_t sample = 0; sample < SENSOR_AVERAGE_SAMPLES; ++sample) {
    total += analogRead(IR_FLAME_PIN);
  }
  const uint16_t flame = total / SENSOR_AVERAGE_SAMPLES;
  // Only present and configured sensors are included. The Pi treats omitted
  // fields as unavailable instead of inventing a normal/safe reading.
  Serial.printf("FLAME:%u", flame);
  if (mpuHeadingReady &&
      static_cast<uint32_t>(micros() - lastMpuSuccessUs) <= MPU_HEADING_FRESH_US) {
    Serial.printf(",HEADING:%.1f", relativeHeadingDegrees);
  }
#if MQ2_ADC_PIN >= 0
  uint32_t gasTotal = 0;
  for (uint8_t sample = 0; sample < SENSOR_AVERAGE_SAMPLES; ++sample) {
    gasTotal += analogRead(MQ2_PIN);
  }
  Serial.printf(",GAS:%u", static_cast<uint16_t>(gasTotal / SENSOR_AVERAGE_SAMPLES));
#endif
  Serial.println();
}

void publishStreamStatus(uint32_t now) {
  if (now - lastStatusMs < STATUS_INTERVAL_MS) return;
  lastStatusMs = now;
  Serial.printf("VSRSTAT:OK=%lu,SHOWN=%lu,CRC=%lu,LEN=%lu,PARTIAL=%lu,LAST=%u\n",
                static_cast<unsigned long>(acceptedFrames),
                static_cast<unsigned long>(displayedFrames),
                static_cast<unsigned long>(crcErrors),
                static_cast<unsigned long>(lengthErrors),
                static_cast<unsigned long>(packetTimeouts), displayedSequence);
}

void scanI2cBus() {
  uint8_t found = 0;
  Serial.print("I2C scan:");
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission(true) == 0) {
      Serial.printf(" 0x%02X", address);
      ++found;
    }
  }
  Serial.printf(" (%u device%s)\n", found, found == 1 ? "" : "s");
}

void setup() {
  Serial.setRxBufferSize(SERIAL_RX_BUFFER_BYTES);
  Serial.begin(SERIAL_BAUD);
  delay(250);
  Serial.println("\nVisor OLED camera receiver VSR1");
  Serial.printf("Transport: USB-UART at %lu 8N1 | Interface: %s | Controller: %s | Mirror: %s\n",
                static_cast<unsigned long>(SERIAL_BAUD), USE_SPI ? "SPI" : "I2C",
                USE_SH1106 ? "SH1106" : "SSD1306", MIRROR_TEXT ? "on" : "off");

  pinMode(MODE_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  pinMode(IR_FLAME_PIN, INPUT);
  analogSetPinAttenuation(IR_FLAME_PIN, ADC_11db);
#if MQ2_ADC_PIN >= 0
  pinMode(MQ2_PIN, INPUT);
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);
#endif
  bool displayBusReady = true;

#if !USE_SPI || ENABLE_MPU6050
  if (startI2cBus()) scanI2cBus();
#endif

#if USE_SPI
  Serial.printf("SPI pins: CLK=%u DATA=%u CS=%u DC=%u RST=%u\n",
                OLED_CLOCK, OLED_DATA, OLED_CS, OLED_DC, OLED_RESET);
#else
  Serial.printf("I2C pins: SDA=%u SCL=%u; requiring OLED at 0x%02X.\n",
                OLED_SDA, OLED_SCL, OLED_I2C_ADDRESS);
  const uint8_t status = i2cBusReady ? probeI2cAddress(OLED_I2C_ADDRESS) : 4;
  if (!i2cBusReady || status != 0) {
    Serial.printf("STOP: OLED 0x%02X did not ACK (I2C status %u).\n",
                  OLED_I2C_ADDRESS, status);
    Serial.println("Check 3V3/GND/SDA/SCL, then reset the ESP32 to retry.");
    displayBusReady = false;
  } else {
    display.setI2CAddress(OLED_I2C_ADDRESS << 1);  // U8g2 uses an 8-bit address.
    display.setBusClock(I2C_CLOCK_HZ);
  }
#endif

  if (displayBusReady) {
    const uint32_t displayInitStartedMs = millis();
    Serial.println("OLED init: begin (optional sensors deferred).");
    displayBusReady = initializeDisplayTransport();
    Serial.printf("OLED init: complete in %lu ms.\n",
                  static_cast<unsigned long>(millis() - displayInitStartedMs));
    if (!displayBusReady) {
      Serial.println("STOP: direct OLED initialization failed.");
    }
    if (display.getBufferTileWidth() != FRAME_WIDTH / 8 ||
        display.getBufferTileHeight() != FRAME_HEIGHT / 8) {
      Serial.println("STOP: display buffer is not 128x64 full-buffer format.");
      displayBusReady = false;
    }
  }
  if (displayBusReady) {
    if (!USE_DIRECT_SSD1306_I2C) display.setContrast(255);
    displayReady = true;
    drawStreamStatus(false);
    displayBusReady = displayReady;
  }

  // The OLED is the primary function. Only touch the optional MPU after the
  // display is initialized, and only if its shared I2C bus actually started.
#if ENABLE_MPU6050
  if (i2cBusReady) {
    beginMpu6050();
  } else {
    Serial.println("MPU6050: skipped because I2C is unavailable; HEADING unavailable.");
  }
#else
  beginMpu6050();
#endif

  lastAnimationMs = millis();
  lastValidFrameMs = lastAnimationMs;
  lastSerialByteMs = lastAnimationMs;
  lastSensorMs = lastAnimationMs;
  lastStatusMs = lastAnimationMs;
  Serial.printf("IR flame sensor: AO=GPIO%u, 12-bit, %lu ms cadence.\n",
                IR_FLAME_PIN, static_cast<unsigned long>(SENSOR_INTERVAL_MS));
#if MQ2_ADC_PIN >= 0
  Serial.printf("MQ-2: divided AO=GPIO%d, 12-bit raw GAS telemetry enabled.\n", MQ2_PIN);
#else
  Serial.println("MQ-2: disabled until its divided AO pin is configured; GAS unavailable.");
#endif
  Serial.println("AIR: unavailable until the temperature sensor model and pin are configured.");
  if (displayReady) {
    Serial.println("READY VSR1: 1024-byte page frames + CRC16/CCITT-FALSE");
    Serial.println("Payload: byte[(y/8)*128+x], bit(y%8), 1=lit; target <=20 FPS.");
  } else {
    Serial.println("READY SENSOR ONLY: flame telemetry remains active without OLED.");
  }
}

void loop() {
  const uint32_t now = millis();
  updateMpu6050(micros());
  publishSensorTelemetry(now);
  publishStreamStatus(now);

  if (!displayReady) {
    // Discard incoming camera packets so a missing OLED cannot make serial
    // backpressure interfere with the independent flame telemetry path.
    while (Serial.available() > 0) Serial.read();
    delay(10);
    return;
  }

  receiveSerialFrames(now);
  updateButton(now);

  if (streamActive && now - lastValidFrameMs > STREAM_TIMEOUT_MS) {
    streamActive = false;
    drawStreamStatus(true);
    Serial.printf("Camera timeout (ok=%lu shown=%lu crc=%lu len=%lu partial=%lu last=%u).\n",
                  static_cast<unsigned long>(acceptedFrames),
                  static_cast<unsigned long>(displayedFrames),
                  static_cast<unsigned long>(crcErrors),
                  static_cast<unsigned long>(lengthErrors),
                  static_cast<unsigned long>(packetTimeouts), displayedSequence);
  }

  if (!streamActive && !fallbackStatusVisible && !alignmentMode &&
      now - lastAnimationMs >= ANIMATION_INTERVAL_MS) {
    lastAnimationMs = now;
    drawAnimationFrame();
  }
  delay(1);
}
