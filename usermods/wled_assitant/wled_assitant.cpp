/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - BME280 temperature / humidity / pressure over WLED's global I2C bus (non-blocking state machine)
 *  - optional MQTT telemetry + Home Assistant discovery (uses WLED's own MQTT client/config)
 *  - Info/JSON diagnostics, including the USB Adalight path
 *
 * LED data is driven straight from the GPIO (3.3V logic) through a 220R series resistor into
 * DIN — no level shifter. The PSU is trimmed to 4.75V so the strip reliably reads 3.3V as a
 * logic high; this usermod has no hardware to control for that.
 *
 * The HyperHDR stream itself is handled by stock WLED (wled00/wled_serial.cpp, Adalight parser
 * reading `Serial`, which is the ESP32-S3 USB-Serial/JTAG "HWCDC" port with the build flags in
 * platformio_override.ini). This usermod only sizes that port's RX queue; it never renders LEDs.
 *
 * All pins come from build flags generated from the [wled_assitant] section of
 * platformio_override.ini — do not hard-code GPIO numbers here.
 */
#include "wled.h"

// ---------------------------------------------------------------------------------------------
// Build-time configuration checks
// ---------------------------------------------------------------------------------------------
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
  #error "wled-assitant targets the ESP32-S3 only"
#endif
#if !ARDUINO_USB_CDC_ON_BOOT || !ARDUINO_USB_MODE
  #error "wled-assitant needs ARDUINO_USB_CDC_ON_BOOT=1 and ARDUINO_USB_MODE=1 so that Serial is the USB-Serial/JTAG (HWCDC) port"
#endif
#ifndef WLED_ENABLE_ADALIGHT
  #error "wled-assitant needs WLED's serial realtime (Adalight) support; remove WLED_DISABLE_ADALIGHT"
#endif
#if defined(WLED_DEBUG) && !defined(WLED_DEBUG_HOST) && !defined(WLEDA_ALLOW_USB_DEBUG)
  #error "WLED_DEBUG would log to the USB port shared with HyperHDR. Use WLED_DEBUG_HOST (UDP log) or define WLEDA_ALLOW_USB_DEBUG."
#endif

#if !defined(WLEDA_LED_DATA_PIN) || !defined(I2CSDAPIN) || !defined(I2CSCLPIN)
  #error "Pin defines missing: build with the [wled_assitant] section of platformio_override.ini"
#endif

#ifndef WLEDA_USB_RX_BUFFER
  #define WLEDA_USB_RX_BUFFER 2048   // bytes; one 144-LED Adalight frame is 438 B
#endif
#ifndef WLEDA_I2C_TIMEOUT_MS
  #define WLEDA_I2C_TIMEOUT_MS 15    // longest transaction (BME280 calibration, 26 B) takes ~3 ms at 100 kHz
#endif

namespace {

// GPIOs that are safe for this project on an ESP32-S3FH4R2 (quad flash + quad PSRAM in package):
// excludes strapping pins (0, 3, 45, 46), USB D-/D+ (19, 20), flash/PSRAM bus (22..32) and
// UART0 (43, 44 — WLED disables serial RX, i.e. Adalight, if GPIO44 is allocated).
constexpr bool s3PinUsable(int p) {
  return (p >= 1 && p <= 18 && p != 3) || p == 21 || (p >= 33 && p <= 42) || p == 47 || p == 48;
}
static_assert(s3PinUsable(WLEDA_LED_DATA_PIN), "LED data pin is reserved/boot-critical on ESP32-S3");
static_assert(s3PinUsable(I2CSDAPIN),          "I2C SDA pin is reserved/boot-critical on ESP32-S3");
static_assert(s3PinUsable(I2CSCLPIN),          "I2C SCL pin is reserved/boot-critical on ESP32-S3");
static_assert(WLEDA_LED_DATA_PIN != I2CSDAPIN && WLEDA_LED_DATA_PIN != I2CSCLPIN && I2CSDAPIN != I2CSCLPIN,
              "LED data, SDA and SCL pins must all be different");

// BME280 (Bosch datasheet BST-BME280-DS002)
constexpr uint8_t  BME_ADDRS[]        = { 0x76, 0x77 };   // SDO low / high
constexpr uint8_t  BME_REG_ID         = 0xD0;
constexpr uint8_t  BME_REG_RESET      = 0xE0;
constexpr uint8_t  BME_REG_CTRL_HUM   = 0xF2;
constexpr uint8_t  BME_REG_STATUS     = 0xF3;
constexpr uint8_t  BME_REG_CTRL_MEAS  = 0xF4;
constexpr uint8_t  BME_REG_CONFIG     = 0xF5;
constexpr uint8_t  BME_REG_DATA       = 0xF7;           // press[3] temp[3] hum[2]
constexpr uint8_t  BME_REG_CALIB_TP   = 0x88;           // 0x88..0xA1: T1..T3, P1..P9, (0xA0 unused), H1
constexpr uint8_t  BME_REG_CALIB_H    = 0xE1;           // 0xE1..0xE7: H2..H6
constexpr uint8_t  BME_CHIP_ID        = 0x60;
constexpr uint8_t  BME_RESET_CMD      = 0xB6;
constexpr uint8_t  BME_STATUS_MEASURING = 0x08;
constexpr uint8_t  BME_STATUS_IM_UPDATE = 0x01;         // NVM calibration still being copied
constexpr uint8_t  BME_OSRS_X1        = 0x01;
// forced mode, temperature and pressure oversampling x1: one conversion per poll, minimal self-heating
constexpr uint8_t  BME_CTRL_MEAS_FORCED = (BME_OSRS_X1 << 5) | (BME_OSRS_X1 << 2) | 0x01;
constexpr uint32_t BME_RESET_MS       = 3;               // datasheet start-up time 2 ms
constexpr uint32_t BME_MEAS_MS        = 10;              // all three channels at x1: max 9.3 ms
constexpr uint32_t BME_POLL_MS        = 5;
constexpr uint8_t  BME_MAX_POLLS      = 5;
constexpr int32_t  BME_SKIPPED_20BIT  = 0x80000;         // register reset value = no conversion done
constexpr int32_t  BME_SKIPPED_16BIT  = 0x8000;

constexpr uint32_t RETRY_AFTER_ERROR_MS = 5000;
constexpr uint8_t  FAILS_BEFORE_MISSING = 3;

struct BmeCalib {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
  uint8_t  H1; int16_t H2; uint8_t H3; int16_t H4, H5; int8_t H6;
};

inline uint16_t le16(const uint8_t* b) { return uint16_t(b[0]) | uint16_t(b[1]) << 8; }

// tp = 26 bytes from 0x88, h = 7 bytes from 0xE1
void bmeParseCalib(BmeCalib& c, const uint8_t* tp, const uint8_t* h) {
  c.T1 = le16(tp + 0);  c.T2 = int16_t(le16(tp + 2));  c.T3 = int16_t(le16(tp + 4));
  c.P1 = le16(tp + 6);  c.P2 = int16_t(le16(tp + 8));  c.P3 = int16_t(le16(tp + 10));
  c.P4 = int16_t(le16(tp + 12)); c.P5 = int16_t(le16(tp + 14)); c.P6 = int16_t(le16(tp + 16));
  c.P7 = int16_t(le16(tp + 18)); c.P8 = int16_t(le16(tp + 20)); c.P9 = int16_t(le16(tp + 22));
  c.H1 = tp[25];
  c.H2 = int16_t(le16(h + 0));
  c.H3 = h[2];
  c.H4 = int16_t(int16_t(int8_t(h[3])) * 16 | (h[4] & 0x0F));   // 0xE4[7:0] << 4 | 0xE5[3:0], signed
  c.H5 = int16_t(int16_t(int8_t(h[5])) * 16 | (h[4] >> 4));     // 0xE6[7:0] << 4 | 0xE5[7:4], signed
  c.H6 = int8_t(h[6]);
}

// Bosch's reference integer compensation (BME280 datasheet 4.2.3 / 8.2), kept verbatim in shape so it
// can be diffed against the datasheet. t_fine from the temperature step feeds pressure and humidity.
int32_t bmeTempCentiC(const BmeCalib& c, int32_t adcT, int32_t& tFine) {
  const int32_t v1 = ((((adcT >> 3) - (int32_t(c.T1) << 1))) * int32_t(c.T2)) >> 11;
  const int32_t v2 = (((((adcT >> 4) - int32_t(c.T1)) * ((adcT >> 4) - int32_t(c.T1))) >> 12) * int32_t(c.T3)) >> 14;
  tFine = v1 + v2;
  return (tFine * 5 + 128) >> 8;
}

uint32_t bmePressQ24_8(const BmeCalib& c, int32_t adcP, int32_t tFine) {   // Pa * 256
  int64_t v1 = int64_t(tFine) - 128000;
  int64_t v2 = v1 * v1 * int64_t(c.P6);
  v2 = v2 + ((v1 * int64_t(c.P5)) << 17);
  v2 = v2 + (int64_t(c.P4) << 35);
  v1 = ((v1 * v1 * int64_t(c.P3)) >> 8) + ((v1 * int64_t(c.P2)) << 12);
  v1 = ((int64_t(1) << 47) + v1) * int64_t(c.P1) >> 33;
  if (v1 == 0) return 0;                       // avoid division by zero (blank calibration)
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - v2) * 3125) / v1;
  v1 = (int64_t(c.P9) * (p >> 13) * (p >> 13)) >> 25;
  v2 = (int64_t(c.P8) * p) >> 19;
  p = ((p + v1 + v2) >> 8) + (int64_t(c.P7) << 4);
  return uint32_t(p);
}

uint32_t bmeHumQ22_10(const BmeCalib& c, int32_t adcH, int32_t tFine) {    // %RH * 1024
  int32_t v = tFine - int32_t(76800);
  v = (((((adcH << 14) - (int32_t(c.H4) << 20) - (int32_t(c.H5) * v)) + int32_t(16384)) >> 15) *
       (((((((v * int32_t(c.H6)) >> 10) * (((v * int32_t(c.H3)) >> 11) + int32_t(32768))) >> 10) +
          int32_t(2097152)) * int32_t(c.H2) + 8192) >> 14));
  v = v - (((((v >> 15) * (v >> 15)) >> 7) * int32_t(c.H1)) >> 4);
  v = v < 0 ? 0 : v;
  v = v > 419430400 ? 419430400 : v;
  return uint32_t(v >> 12);
}

inline bool timeReached(uint32_t now, uint32_t at) { return int32_t(now - at) >= 0; }
inline float round1(float v) { return roundf(v * 10.0f) / 10.0f; }

} // namespace


class WledAssitant : public Usermod {
  public:
    // Static construction runs before Arduino setup() and therefore before WLED touches any GPIO
    // or opens Serial — the earliest hook reachable without patching WLED core.
    WledAssitant() {
      // Stock HWCDC RX queue is 256 B — smaller than one Adalight frame — and the HWCDC ISR drops
      // bytes when it is full. HWCDC::begin() (called later by WLED) keeps a pre-sized queue, so
      // resizing here avoids replacing the queue while the USB ISR is live.
      Serial.setRxBufferSize(WLEDA_USB_RX_BUFFER);
    }

    void setup() override {
      if (i2c_sda >= 0 && i2c_scl >= 0) Wire.setTimeOut(WLEDA_I2C_TIMEOUT_MS);
      nextPollAt = millis() + 2000;      // first reading shortly after boot, off the critical path
      initDone = true;
    }

    void loop() override {
      const uint32_t now = millis();
      if (bmeEnabled) serviceBme(now);
#ifndef WLED_DISABLE_MQTT
      if (discoveryDirty && WLED_MQTT_CONNECTED) {
        discoveryDirty = false;
        publishDiscovery();
        publishAvailability();
        publishReadings();
      }
#endif
    }

#ifndef WLED_DISABLE_MQTT
    // Runs in the AsyncMqttClient (network) task: only flag the work, loop() publishes.
    void onMqttConnect(bool sessionPresent) override { discoveryDirty = true; }
#endif

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      const bool fresh = readingFresh(millis());
      JsonArray t = user.createNestedArray(F("Temperature"));
      JsonArray h = user.createNestedArray(F("Humidity"));
      JsonArray p = user.createNestedArray(F("Pressure"));
      if (fresh) {
        t.add(round1(temperatureC)); t.add(F(" °C"));
        h.add(round1(humidityRH));   h.add(F(" %RH"));
        p.add(round1(pressureHPa));  p.add(F(" hPa"));
      } else {
        t.add(sensorStatusText());
        h.add(sensorStatusText());
        p.add(sensorStatusText());
      }

      JsonArray usb = user.createNestedArray(F("USB Adalight"));
      usb.add(usbStatusText());

      // machine-readable copy for scripts / HA REST sensors
      JsonObject d = root.createNestedObject(F("wled_assitant"));
      JsonObject s = d.createNestedObject(F("bme280"));
      s[F("status")] = sensorStatusText();
      if (bmeAddr) s[F("addr")] = bmeAddr;
      if (fresh) {
        s[F("temp_c")] = round1(temperatureC);
        s[F("rh_pct")] = round1(humidityRH);
        s[F("hpa")]    = round1(pressureHPa);
        s[F("age_s")]  = (millis() - lastReadingMs) / 1000;
        s[F("raw_t")]  = rawTemp;
        s[F("raw_rh")] = rawHum;
        s[F("raw_p")]  = rawPress;
      }
      s[F("errors")] = errorCount;
      if (lastFault[0]) s[F("last_fault")] = lastFault;
      JsonObject u = d.createNestedObject(F("usb"));
      u[F("adalight_rx")] = serialCanRX;
      u[F("streaming")]   = realtimeMode == REALTIME_MODE_ADALIGHT;
      u[F("rx_buffer")]   = WLEDA_USB_RX_BUFFER;
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[F("bmeEnabled")]      = bmeEnabled;
      top[F("interval")]        = intervalSec;
      top[F("tempOffset")]      = tempOffset;
      top[F("humOffset")]       = humOffset;
      top[F("mqttPublish")]     = mqttPublish;
      top[F("haDiscovery")]     = haDiscovery;
      top[F("discoveryPrefix")] = discoveryPrefix;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool complete = !top.isNull();
      complete &= getJsonValue(top[F("bmeEnabled")],  bmeEnabled,  true);
      complete &= getJsonValue(top[F("interval")],    intervalSec, uint16_t(30));
      complete &= getJsonValue(top[F("tempOffset")],  tempOffset,  0.0f);
      complete &= getJsonValue(top[F("humOffset")],   humOffset,   0.0f);
      complete &= getJsonValue(top[F("mqttPublish")], mqttPublish, true);
      complete &= getJsonValue(top[F("haDiscovery")], haDiscovery, true);
      const char* prefix = top[F("discoveryPrefix")] | "homeassistant";
      strlcpy(discoveryPrefix, prefix[0] ? prefix : "homeassistant", sizeof(discoveryPrefix));
      complete &= !top[F("discoveryPrefix")].isNull();

      intervalSec = constrain(intervalSec, 5, 3600);

      if (initDone) {
        if (!bmeEnabled) {
          bmeState   = BmeState::Idle;
          bmeStatus  = BmeStatus::NotRead;
          bmePresent = false;
        }
        nextPollAt = millis();          // apply new settings with a fresh reading
        discoveryDirty = true;          // republish (or clear) HA discovery
      }
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("addInfo('wled_assitant:interval',1,'s (5-3600)');"));
      s.print(F("addInfo('wled_assitant:tempOffset',1,'°C');"));
      s.print(F("addInfo('wled_assitant:humOffset',1,'%RH');"));
      s.print(F("addInfo('wled_assitant:mqttPublish',1,'uses the MQTT settings in Sync Interfaces; no broker is fine');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  private:
    static const char _name[];

    // ---- settings (cfg.json "um" → "wled_assitant") ----
    bool     bmeEnabled  = true;
    uint16_t intervalSec = 30;
    float    tempOffset  = 0.0f;
    float    humOffset   = 0.0f;
    bool     mqttPublish = true;
    bool     haDiscovery = true;
    char     discoveryPrefix[33] = "homeassistant";

    bool     initDone    = false;

    // ---- BME280 state ----
    enum class BmeState : uint8_t { Idle, Resetting, Measuring };
    enum class BmeStatus : uint8_t { NotRead, NoBus, NotFound, ReadError, Ok };
    BmeState  bmeState   = BmeState::Idle;
    BmeStatus bmeStatus  = BmeStatus::NotRead;
    bool      bmePresent = false;
    uint8_t   bmeAddr    = 0;      // 0x76 or 0x77 once found
    uint8_t   polls      = 0;
    uint8_t   consecutiveFails = 0;
    uint32_t  errorCount = 0;
    uint32_t  deadline   = 0;
    uint32_t  nextPollAt = 0;
    int32_t   rawTemp    = 0;
    int32_t   rawHum     = 0;
    int32_t   rawPress   = 0;
    char      lastFault[24] = "";   // why the last read failed, e.g. "id 58 @76"; diagnostics only
    uint32_t  lastReadingMs = 0;
    float     temperatureC = NAN;
    float     humidityRH   = NAN;
    float     pressureHPa  = NAN;
    BmeCalib  calib = {};

    volatile bool discoveryDirty = false;   // set from the MQTT task
    bool lastPublishedPresent = false;

    const __FlashStringHelper* usbStatusText() const {
      if (!serialCanRX) return F("disabled (GPIO44/RX is allocated)");
      if (realtimeMode == REALTIME_MODE_ADALIGHT) return F("streaming");
      return F("idle");
    }

    // ------------------------------------------------------------------------------------------
    // BME280 — forced mode, one short I2C transaction per step; conversions are waited out across
    // loop() calls and the sensor sleeps between polls
    // ------------------------------------------------------------------------------------------
    static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
      Wire.beginTransmission(addr);
      Wire.write(reg);
      Wire.write(value);
      return Wire.endTransmission() == 0;
    }

    static bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
      Wire.beginTransmission(addr);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return false;    // repeated start, as the BME280 expects
      if (Wire.requestFrom(addr, len) != len) {
        while (Wire.available()) Wire.read();
        return false;
      }
      for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
      return true;
    }

    void bmeFail(uint32_t now, BmeStatus why) {
      errorCount++;
      bmeState = BmeState::Idle;
      if (why == BmeStatus::NotFound || ++consecutiveFails >= FAILS_BEFORE_MISSING) {
        bmePresent = false;              // next poll probes again (reset + calibration)
        bmeStatus  = why;
        nextPollAt = now + uint32_t(intervalSec) * 1000;
      } else {
        nextPollAt = now + RETRY_AFTER_ERROR_MS;
      }
      publishAvailabilityIfChanged();
    }

    // The chip ID check rejects BMP280s (0x56-0x58), which are often sold as "BME280" but have no
    // humidity sensor; the fault names the ID so the wrong part is obvious.
    void bmeProbe(uint32_t now) {
      for (uint8_t addr : BME_ADDRS) {
        uint8_t id;
        if (!readRegs(addr, BME_REG_ID, &id, 1)) continue;
        if (id != BME_CHIP_ID) {
          snprintf_P(lastFault, sizeof(lastFault), PSTR("id %02X @%02X, not BME280"), id, addr);
          continue;
        }
        bmeAddr = addr;
        if (!writeReg(addr, BME_REG_RESET, BME_RESET_CMD)) { bmeFail(now, BmeStatus::ReadError); return; }
        bmeState = BmeState::Resetting;
        deadline = now + BME_RESET_MS;
        polls = 0;
        return;
      }
      bmeAddr = 0;
      bmeFail(now, BmeStatus::NotFound);
    }

    void serviceBme(uint32_t now) {
      if (i2c_sda < 0 || i2c_scl < 0) { bmeStatus = BmeStatus::NoBus; return; }

      switch (bmeState) {
        case BmeState::Idle:
          if (!timeReached(now, nextPollAt)) return;
          if (bmePresent) bmeStartMeasurement(now);
          else            bmeProbe(now);
          return;
        case BmeState::Resetting: bmeLoadCalibration(now); return;
        case BmeState::Measuring: bmeReadMeasurement(now); return;
      }
    }

    void bmeLoadCalibration(uint32_t now) {
      if (!timeReached(now, deadline)) return;
      uint8_t status;
      if (!readRegs(bmeAddr, BME_REG_STATUS, &status, 1)) { bmeFail(now, BmeStatus::ReadError); return; }
      if ((status & BME_STATUS_IM_UPDATE) && ++polls < BME_MAX_POLLS) { deadline = now + BME_POLL_MS; return; }

      uint8_t tp[26], h[7];
      if (!readRegs(bmeAddr, BME_REG_CALIB_TP, tp, sizeof(tp)) || !readRegs(bmeAddr, BME_REG_CALIB_H, h, sizeof(h))) {
        strcpy_P(lastFault, PSTR("calibration read"));
        bmeFail(now, BmeStatus::ReadError);
        return;
      }
      bmeParseCalib(calib, tp, h);
      if (calib.T1 == 0 || calib.P1 == 0) {        // blank NVM / bus glitch: compensation would be garbage
        strcpy_P(lastFault, PSTR("calibration blank"));
        bmeFail(now, BmeStatus::ReadError);
        return;
      }
      if (!writeReg(bmeAddr, BME_REG_CONFIG, 0x00)) { bmeFail(now, BmeStatus::ReadError); return; }   // filter off
      bmeStartMeasurement(now);
    }

    void bmeStartMeasurement(uint32_t now) {
      // ctrl_hum only takes effect with the following ctrl_meas write
      if (!writeReg(bmeAddr, BME_REG_CTRL_HUM, BME_OSRS_X1) ||
          !writeReg(bmeAddr, BME_REG_CTRL_MEAS, BME_CTRL_MEAS_FORCED)) {
        bmeFail(now, BmeStatus::NotFound);
        return;
      }
      bmeState = BmeState::Measuring;
      deadline = now + BME_MEAS_MS;
      polls = 0;
    }

    void bmeReadMeasurement(uint32_t now) {
      if (!timeReached(now, deadline)) return;
      uint8_t status;
      if (!readRegs(bmeAddr, BME_REG_STATUS, &status, 1)) { bmeFail(now, BmeStatus::ReadError); return; }
      if (status & BME_STATUS_MEASURING) {
        if (++polls < BME_MAX_POLLS) { deadline = now + BME_POLL_MS; return; }
        strcpy_P(lastFault, PSTR("conversion timeout"));
        bmeFail(now, BmeStatus::ReadError);
        return;
      }

      uint8_t b[8];
      if (!readRegs(bmeAddr, BME_REG_DATA, b, sizeof(b))) { bmeFail(now, BmeStatus::ReadError); return; }
      const int32_t adcP = int32_t(b[0]) << 12 | int32_t(b[1]) << 4 | b[2] >> 4;
      const int32_t adcT = int32_t(b[3]) << 12 | int32_t(b[4]) << 4 | b[5] >> 4;
      const int32_t adcH = int32_t(b[6]) << 8 | b[7];
      // registers keep their reset value when a conversion was skipped or the sensor rebooted
      if (adcT == BME_SKIPPED_20BIT || adcP == BME_SKIPPED_20BIT || adcH == BME_SKIPPED_16BIT) {
        snprintf_P(lastFault, sizeof(lastFault), PSTR("skipped %05lX %05lX"), long(adcT), long(adcP));
        bmeFail(now, BmeStatus::ReadError);
        return;
      }

      int32_t tFine;
      const int32_t centiC = bmeTempCentiC(calib, adcT, tFine);
      rawTemp = adcT; rawPress = adcP; rawHum = adcH;
      temperatureC = centiC / 100.0f + tempOffset;
      pressureHPa  = bmePressQ24_8(calib, adcP, tFine) / 25600.0f;
      humidityRH   = constrain(bmeHumQ22_10(calib, adcH, tFine) / 1024.0f + humOffset, 0.0f, 100.0f);

      lastReadingMs = now;
      bmePresent = true;
      bmeStatus  = BmeStatus::Ok;
      consecutiveFails = 0;
      bmeState   = BmeState::Idle;
      nextPollAt = now + uint32_t(intervalSec) * 1000;
      publishAvailabilityIfChanged();
      publishReadings();
    }

    bool readingFresh(uint32_t now) const {
      return lastReadingMs != 0 && bmeStatus == BmeStatus::Ok &&
             (now - lastReadingMs) < 3UL * intervalSec * 1000;
    }

    const __FlashStringHelper* sensorStatusText() const {
      if (!bmeEnabled) return F("disabled");
      switch (bmeStatus) {
        case BmeStatus::NoBus:     return F("no I2C bus - set SDA/SCL in Usermods settings");
        case BmeStatus::NotFound:  return F("BME280 not found at 0x76/0x77");
        case BmeStatus::ReadError: return F("read error - see last_fault");
        case BmeStatus::Ok:        return F("ok");
        default:                   return F("not read yet");
      }
    }

    // ------------------------------------------------------------------------------------------
    // MQTT / Home Assistant discovery — strictly optional; every call is a no-op without a broker
    // ------------------------------------------------------------------------------------------
#ifndef WLED_DISABLE_MQTT
    static size_t jsonEscape(char* dst, const char* src, size_t size) {
      size_t o = 0;
      for (; *src && o + 2 < size; src++) {
        const char c = *src;
        if (c == '"' || c == '\\') dst[o++] = '\\';
        if (uint8_t(c) >= 0x20) dst[o++] = c;
      }
      dst[o] = '\0';
      return o;
    }

    bool mqttReady() const { return mqttPublish && WLED_MQTT_CONNECTED && mqttDeviceTopic[0]; }

    void topicFor(char* buf, size_t len, const char* leaf) const {
      snprintf_P(buf, len, PSTR("%s/bme280/%s"), mqttDeviceTopic, leaf);
    }

    void publishDiscoveryEntity(const char* key, const char* name, const char* unit, const char* devClass) {
      static char topic[128];
      static char payload[1024];   // static: no per-publish heap churn
      snprintf_P(topic, sizeof(topic), PSTR("%s/sensor/wled_%s/%s/config"), discoveryPrefix, escapedMac.c_str(), key);

      if (!haDiscovery || !mqttPublish) {        // remove previously announced entity
        mqtt->publish(topic, 0, true, "");
        return;
      }

      char devName[70];
      jsonEscape(devName, serverDescription, sizeof(devName));
      const char* m = escapedMac.c_str();         // "aabbccddeeff"
      snprintf_P(payload, sizeof(payload), PSTR(
        "{\"name\":\"%s\",\"uniq_id\":\"wled_%s_%s\",\"obj_id\":\"wled_%s_%s\","
        "\"stat_t\":\"%s/bme280/%s\",\"unit_of_meas\":\"%s\",\"dev_cla\":\"%s\",\"stat_cla\":\"measurement\","
        "\"sug_dsp_prec\":1,\"avty_mode\":\"all\",\"avty\":[{\"t\":\"%s/status\"},{\"t\":\"%s/bme280/status\"}],"
        "\"dev\":{\"cns\":[[\"mac\",\"%.2s:%.2s:%.2s:%.2s:%.2s:%.2s\"]],\"name\":\"%s\",\"mf\":\"WLED\","
        "\"mdl\":\"ESP32-S3 SuperMini + BME280\",\"sw\":\"%s\"}}"),
        name, m, key, m, key,
        mqttDeviceTopic, key, unit, devClass,
        mqttDeviceTopic, mqttDeviceTopic,
        m, m + 2, m + 4, m + 6, m + 8, m + 10, devName,
        versionString);
      mqtt->publish(topic, 0, true, payload);
    }

    void publishDiscovery() {
      if (!WLED_MQTT_CONNECTED || !mqttDeviceTopic[0] || escapedMac.length() < 12) return;
      publishDiscoveryEntity("temperature", "Temperature", "°C",  "temperature");
      publishDiscoveryEntity("humidity",    "Humidity",    "%",   "humidity");
      publishDiscoveryEntity("pressure",    "Pressure",    "hPa", "atmospheric_pressure");
    }

    void publishAvailability() {
      if (!mqttReady()) return;
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      topicFor(topic, sizeof(topic), "status");
      mqtt->publish(topic, 0, true, bmePresent ? "online" : "offline");
      lastPublishedPresent = bmePresent;
    }

    void publishAvailabilityIfChanged() {
      if (bmePresent != lastPublishedPresent) publishAvailability();
    }

    void publishValue(const char* leaf, float v) {
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      char value[16];
      topicFor(topic, sizeof(topic), leaf);
      snprintf_P(value, sizeof(value), PSTR("%.1f"), v);
      mqtt->publish(topic, 0, true, value);
    }

    void publishReadings() {
      if (!mqttReady() || !readingFresh(millis())) return;
      publishValue("temperature", temperatureC);
      publishValue("humidity",    humidityRH);
      publishValue("pressure",    pressureHPa);
    }
#else
    void publishAvailabilityIfChanged() {}
    void publishReadings() {}
#endif
};

const char WledAssitant::_name[] PROGMEM = "wled_assitant";

static WledAssitant wled_assitant;
REGISTER_USERMOD(wled_assitant);
