/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - BMP280 temperature / pressure over WLED's global I2C bus (non-blocking state machine)
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
  #define WLEDA_I2C_TIMEOUT_MS 15    // longest transaction (BMP280 calibration, 24 B) takes ~3 ms at 100 kHz
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

// BMP280 (Bosch datasheet BST-BMP280-DS001)
constexpr uint8_t  BMP_ADDRS[]        = { 0x76, 0x77 };   // SDO low / high
constexpr uint8_t  BMP_REG_ID         = 0xD0;
constexpr uint8_t  BMP_REG_RESET      = 0xE0;
constexpr uint8_t  BMP_REG_STATUS     = 0xF3;
constexpr uint8_t  BMP_REG_CTRL_MEAS  = 0xF4;
constexpr uint8_t  BMP_REG_CONFIG     = 0xF5;
constexpr uint8_t  BMP_REG_DATA       = 0xF7;           // press[3] temp[3]
constexpr uint8_t  BMP_REG_CALIB_TP   = 0x88;           // 0x88..0x9F: T1..T3, P1..P9
constexpr uint8_t  BMP_RESET_CMD      = 0xB6;
constexpr uint8_t  BMP_STATUS_MEASURING = 0x08;
constexpr uint8_t  BMP_STATUS_IM_UPDATE = 0x01;         // NVM calibration still being copied
constexpr uint8_t  BMP_OSRS_X1        = 0x01;
// forced mode, temperature and pressure oversampling x1: one conversion per poll, minimal self-heating
constexpr uint8_t  BMP_CTRL_MEAS_FORCED = (BMP_OSRS_X1 << 5) | (BMP_OSRS_X1 << 2) | 0x01;
constexpr uint32_t BMP_RESET_MS       = 3;               // datasheet start-up time 2 ms
constexpr uint32_t BMP_MEAS_MS        = 8;               // temperature + pressure at x1: max 6.4 ms
constexpr uint32_t BMP_POLL_MS        = 5;
constexpr uint8_t  BMP_MAX_POLLS      = 5;
constexpr int32_t  BMP_SKIPPED_20BIT  = 0x80000;         // register reset value = no conversion done

constexpr bool isBmp280Id(uint8_t id) { return id >= 0x56 && id <= 0x58; }   // 0x56/0x57 samples, 0x58 mass production

constexpr uint32_t RETRY_AFTER_ERROR_MS = 5000;
constexpr uint8_t  FAILS_BEFORE_MISSING = 3;

struct BmpCalib {
  uint16_t T1; int16_t T2, T3;
  uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
};

inline uint16_t le16(const uint8_t* b) { return uint16_t(b[0]) | uint16_t(b[1]) << 8; }

// tp = 24 bytes from 0x88
void bmpParseCalib(BmpCalib& c, const uint8_t* tp) {
  c.T1 = le16(tp + 0);  c.T2 = int16_t(le16(tp + 2));  c.T3 = int16_t(le16(tp + 4));
  c.P1 = le16(tp + 6);  c.P2 = int16_t(le16(tp + 8));  c.P3 = int16_t(le16(tp + 10));
  c.P4 = int16_t(le16(tp + 12)); c.P5 = int16_t(le16(tp + 14)); c.P6 = int16_t(le16(tp + 16));
  c.P7 = int16_t(le16(tp + 18)); c.P8 = int16_t(le16(tp + 20)); c.P9 = int16_t(le16(tp + 22));
}

// Bosch's reference integer compensation (BMP280 datasheet 3.11.3), kept verbatim in shape so it
// can be diffed against the datasheet. t_fine from the temperature step feeds pressure.
int32_t bmpTempCentiC(const BmpCalib& c, int32_t adcT, int32_t& tFine) {
  const int32_t v1 = ((((adcT >> 3) - (int32_t(c.T1) << 1))) * int32_t(c.T2)) >> 11;
  const int32_t v2 = (((((adcT >> 4) - int32_t(c.T1)) * ((adcT >> 4) - int32_t(c.T1))) >> 12) * int32_t(c.T3)) >> 14;
  tFine = v1 + v2;
  return (tFine * 5 + 128) >> 8;
}

uint32_t bmpPressQ24_8(const BmpCalib& c, int32_t adcP, int32_t tFine) {   // Pa * 256
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
      if (bmpEnabled) serviceBmp(now);
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
      JsonArray p = user.createNestedArray(F("Pressure"));
      if (fresh) {
        t.add(round1(temperatureC)); t.add(F(" °C"));
        p.add(round1(pressureHPa));  p.add(F(" hPa"));
      } else {
        t.add(sensorStatusText());
        p.add(sensorStatusText());
      }

      JsonArray usb = user.createNestedArray(F("USB Adalight"));
      usb.add(usbStatusText());

      // machine-readable copy for scripts / HA REST sensors
      JsonObject d = root.createNestedObject(F("wled_assitant"));
      JsonObject s = d.createNestedObject(F("bmp280"));
      s[F("status")] = sensorStatusText();
      if (bmpAddr) s[F("addr")] = bmpAddr;
      if (fresh) {
        s[F("temp_c")] = round1(temperatureC);
        s[F("hpa")]    = round1(pressureHPa);
        s[F("age_s")]  = (millis() - lastReadingMs) / 1000;
        s[F("raw_t")]  = rawTemp;
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
      top[F("bmpEnabled")]      = bmpEnabled;
      top[F("interval")]        = intervalSec;
      top[F("tempOffset")]      = tempOffset;
      top[F("mqttPublish")]     = mqttPublish;
      top[F("haDiscovery")]     = haDiscovery;
      top[F("discoveryPrefix")] = discoveryPrefix;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool complete = !top.isNull();
      complete &= getJsonValue(top[F("bmpEnabled")],  bmpEnabled,  true);
      complete &= getJsonValue(top[F("interval")],    intervalSec, uint16_t(30));
      complete &= getJsonValue(top[F("tempOffset")],  tempOffset,  0.0f);
      complete &= getJsonValue(top[F("mqttPublish")], mqttPublish, true);
      complete &= getJsonValue(top[F("haDiscovery")], haDiscovery, true);
      const char* prefix = top[F("discoveryPrefix")] | "homeassistant";
      strlcpy(discoveryPrefix, prefix[0] ? prefix : "homeassistant", sizeof(discoveryPrefix));
      complete &= !top[F("discoveryPrefix")].isNull();

      intervalSec = constrain(intervalSec, 5, 3600);

      if (initDone) {
        if (!bmpEnabled) {
          bmpState   = BmpState::Idle;
          bmpStatus  = BmpStatus::NotRead;
          bmpPresent = false;
        }
        nextPollAt = millis();          // apply new settings with a fresh reading
        discoveryDirty = true;          // republish (or clear) HA discovery
      }
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("addInfo('wled_assitant:interval',1,'s (5-3600)');"));
      s.print(F("addInfo('wled_assitant:tempOffset',1,'°C');"));
      s.print(F("addInfo('wled_assitant:mqttPublish',1,'uses the MQTT settings in Sync Interfaces; no broker is fine');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  private:
    static const char _name[];

    // ---- settings (cfg.json "um" → "wled_assitant") ----
    bool     bmpEnabled  = true;
    uint16_t intervalSec = 30;
    float    tempOffset  = 0.0f;
    bool     mqttPublish = true;
    bool     haDiscovery = true;
    char     discoveryPrefix[33] = "homeassistant";

    bool     initDone    = false;

    // ---- BMP280 state ----
    enum class BmpState : uint8_t { Idle, Resetting, Measuring };
    enum class BmpStatus : uint8_t { NotRead, NoBus, NotFound, ReadError, Ok };
    BmpState  bmpState   = BmpState::Idle;
    BmpStatus bmpStatus  = BmpStatus::NotRead;
    bool      bmpPresent = false;
    uint8_t   bmpAddr    = 0;      // 0x76 or 0x77 once found
    uint8_t   polls      = 0;
    uint8_t   consecutiveFails = 0;
    uint32_t  errorCount = 0;
    uint32_t  deadline   = 0;
    uint32_t  nextPollAt = 0;
    int32_t   rawTemp    = 0;
    int32_t   rawPress   = 0;
    char      lastFault[24] = "";   // why the last read failed, e.g. "id 58 @76"; diagnostics only
    uint32_t  lastReadingMs = 0;
    float     temperatureC = NAN;
    float     pressureHPa  = NAN;
    BmpCalib  calib = {};

    volatile bool discoveryDirty = false;   // set from the MQTT task
    bool lastPublishedPresent = false;

    const __FlashStringHelper* usbStatusText() const {
      if (!serialCanRX) return F("disabled (GPIO44/RX is allocated)");
      if (realtimeMode == REALTIME_MODE_ADALIGHT) return F("streaming");
      return F("idle");
    }

    // ------------------------------------------------------------------------------------------
    // BMP280 — forced mode, one short I2C transaction per step; conversions are waited out across
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
      if (Wire.endTransmission(false) != 0) return false;    // repeated start, as the BMP280 expects
      if (Wire.requestFrom(addr, len) != len) {
        while (Wire.available()) Wire.read();
        return false;
      }
      for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
      return true;
    }

    void bmpFail(uint32_t now, BmpStatus why) {
      errorCount++;
      bmpState = BmpState::Idle;
      if (why == BmpStatus::NotFound || ++consecutiveFails >= FAILS_BEFORE_MISSING) {
        bmpPresent = false;              // next poll probes again (reset + calibration)
        bmpStatus  = why;
        nextPollAt = now + uint32_t(intervalSec) * 1000;
      } else {
        nextPollAt = now + RETRY_AFTER_ERROR_MS;
      }
      publishAvailabilityIfChanged();
    }

    // The fault names an unexpected chip ID so a wrong part is obvious.
    void bmpProbe(uint32_t now) {
      for (uint8_t addr : BMP_ADDRS) {
        uint8_t id;
        if (!readRegs(addr, BMP_REG_ID, &id, 1)) continue;
        if (!isBmp280Id(id)) {
          snprintf_P(lastFault, sizeof(lastFault), PSTR("id %02X @%02X, not BMP280"), id, addr);
          continue;
        }
        bmpAddr = addr;
        if (!writeReg(addr, BMP_REG_RESET, BMP_RESET_CMD)) { bmpFail(now, BmpStatus::ReadError); return; }
        bmpState = BmpState::Resetting;
        deadline = now + BMP_RESET_MS;
        polls = 0;
        return;
      }
      bmpAddr = 0;
      bmpFail(now, BmpStatus::NotFound);
    }

    void serviceBmp(uint32_t now) {
      if (i2c_sda < 0 || i2c_scl < 0) { bmpStatus = BmpStatus::NoBus; return; }

      switch (bmpState) {
        case BmpState::Idle:
          if (!timeReached(now, nextPollAt)) return;
          if (bmpPresent) bmpStartMeasurement(now);
          else            bmpProbe(now);
          return;
        case BmpState::Resetting: bmpLoadCalibration(now); return;
        case BmpState::Measuring: bmpReadMeasurement(now); return;
      }
    }

    void bmpLoadCalibration(uint32_t now) {
      if (!timeReached(now, deadline)) return;
      uint8_t status;
      if (!readRegs(bmpAddr, BMP_REG_STATUS, &status, 1)) { bmpFail(now, BmpStatus::ReadError); return; }
      if ((status & BMP_STATUS_IM_UPDATE) && ++polls < BMP_MAX_POLLS) { deadline = now + BMP_POLL_MS; return; }

      uint8_t tp[24];
      if (!readRegs(bmpAddr, BMP_REG_CALIB_TP, tp, sizeof(tp))) {
        strcpy_P(lastFault, PSTR("calibration read"));
        bmpFail(now, BmpStatus::ReadError);
        return;
      }
      bmpParseCalib(calib, tp);
      if (calib.T1 == 0 || calib.P1 == 0) {        // blank NVM / bus glitch: compensation would be garbage
        strcpy_P(lastFault, PSTR("calibration blank"));
        bmpFail(now, BmpStatus::ReadError);
        return;
      }
      if (!writeReg(bmpAddr, BMP_REG_CONFIG, 0x00)) { bmpFail(now, BmpStatus::ReadError); return; }   // filter off
      bmpStartMeasurement(now);
    }

    void bmpStartMeasurement(uint32_t now) {
      if (!writeReg(bmpAddr, BMP_REG_CTRL_MEAS, BMP_CTRL_MEAS_FORCED)) {
        bmpFail(now, BmpStatus::NotFound);
        return;
      }
      bmpState = BmpState::Measuring;
      deadline = now + BMP_MEAS_MS;
      polls = 0;
    }

    void bmpReadMeasurement(uint32_t now) {
      if (!timeReached(now, deadline)) return;
      uint8_t status;
      if (!readRegs(bmpAddr, BMP_REG_STATUS, &status, 1)) { bmpFail(now, BmpStatus::ReadError); return; }
      if (status & BMP_STATUS_MEASURING) {
        if (++polls < BMP_MAX_POLLS) { deadline = now + BMP_POLL_MS; return; }
        strcpy_P(lastFault, PSTR("conversion timeout"));
        bmpFail(now, BmpStatus::ReadError);
        return;
      }

      uint8_t b[6];
      if (!readRegs(bmpAddr, BMP_REG_DATA, b, sizeof(b))) { bmpFail(now, BmpStatus::ReadError); return; }
      const int32_t adcP = int32_t(b[0]) << 12 | int32_t(b[1]) << 4 | b[2] >> 4;
      const int32_t adcT = int32_t(b[3]) << 12 | int32_t(b[4]) << 4 | b[5] >> 4;
      // registers keep their reset value when a conversion was skipped or the sensor rebooted
      if (adcT == BMP_SKIPPED_20BIT || adcP == BMP_SKIPPED_20BIT) {
        snprintf_P(lastFault, sizeof(lastFault), PSTR("skipped %05lX %05lX"), long(adcT), long(adcP));
        bmpFail(now, BmpStatus::ReadError);
        return;
      }

      int32_t tFine;
      const int32_t centiC = bmpTempCentiC(calib, adcT, tFine);
      rawTemp = adcT; rawPress = adcP;
      temperatureC = centiC / 100.0f + tempOffset;
      pressureHPa  = bmpPressQ24_8(calib, adcP, tFine) / 25600.0f;

      lastReadingMs = now;
      bmpPresent = true;
      bmpStatus  = BmpStatus::Ok;
      consecutiveFails = 0;
      bmpState   = BmpState::Idle;
      nextPollAt = now + uint32_t(intervalSec) * 1000;
      publishAvailabilityIfChanged();
      publishReadings();
    }

    bool readingFresh(uint32_t now) const {
      return lastReadingMs != 0 && bmpStatus == BmpStatus::Ok &&
             (now - lastReadingMs) < 3UL * intervalSec * 1000;
    }

    const __FlashStringHelper* sensorStatusText() const {
      if (!bmpEnabled) return F("disabled");
      switch (bmpStatus) {
        case BmpStatus::NoBus:     return F("no I2C bus - set SDA/SCL in Usermods settings");
        case BmpStatus::NotFound:  return F("BMP280 not found at 0x76/0x77");
        case BmpStatus::ReadError: return F("read error - see last_fault");
        case BmpStatus::Ok:        return F("ok");
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
      snprintf_P(buf, len, PSTR("%s/bmp280/%s"), mqttDeviceTopic, leaf);
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
        "\"stat_t\":\"%s/bmp280/%s\",\"unit_of_meas\":\"%s\",\"dev_cla\":\"%s\",\"stat_cla\":\"measurement\","
        "\"sug_dsp_prec\":1,\"avty_mode\":\"all\",\"avty\":[{\"t\":\"%s/status\"},{\"t\":\"%s/bmp280/status\"}],"
        "\"dev\":{\"cns\":[[\"mac\",\"%.2s:%.2s:%.2s:%.2s:%.2s:%.2s\"]],\"name\":\"%s\",\"mf\":\"WLED\","
        "\"mdl\":\"ESP32-S3 SuperMini + BMP280\",\"sw\":\"%s\"}}"),
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
      publishDiscoveryEntity("pressure",    "Pressure",    "hPa", "atmospheric_pressure");
    }

    void publishAvailability() {
      if (!mqttReady()) return;
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      topicFor(topic, sizeof(topic), "status");
      mqtt->publish(topic, 0, true, bmpPresent ? "online" : "offline");
      lastPublishedPresent = bmpPresent;
    }

    void publishAvailabilityIfChanged() {
      if (bmpPresent != lastPublishedPresent) publishAvailability();
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
