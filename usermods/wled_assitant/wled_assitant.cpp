/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - HTU21D temperature / humidity over WLED's global I2C bus (non-blocking state machine)
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
  #define WLEDA_I2C_TIMEOUT_MS 15    // a full HTU21D transaction takes < 1 ms at 100 kHz
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

// HTU21D / SHT21 / Si7021-compatible command set
constexpr uint8_t HTU21D_ADDR          = 0x40;
constexpr uint8_t HTU21D_TRIG_T_NOHOLD = 0xF3;
constexpr uint8_t HTU21D_TRIG_H_NOHOLD = 0xF5;
constexpr uint8_t HTU21D_SOFT_RESET    = 0xFE;
constexpr uint32_t HTU21D_RESET_MS     = 15;  // datasheet: < 15 ms
constexpr uint32_t HTU21D_T_CONV_MS    = 55;  // 14-bit temperature: max 50 ms
constexpr uint32_t HTU21D_H_CONV_MS    = 20;  // 12-bit humidity:    max 16 ms
constexpr uint32_t HTU21D_POLL_MS      = 10;  // re-poll while the sensor NACKs (still converting)
constexpr uint8_t  HTU21D_MAX_POLLS    = 5;
constexpr uint32_t RETRY_AFTER_ERROR_MS = 5000;
constexpr uint8_t  FAILS_BEFORE_MISSING = 3;

uint8_t htuCrc8(uint8_t msb, uint8_t lsb) {
  // CRC-8, polynomial x^8 + x^5 + x^4 + 1 (0x31), init 0x00
  uint8_t crc = 0;
  const uint8_t data[2] = { msb, lsb };
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? uint8_t((crc << 1) ^ 0x31) : uint8_t(crc << 1);
  }
  return crc;
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
      if (htuEnabled) serviceHtu(now);
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
      if (fresh) {
        t.add(round1(temperatureC)); t.add(F(" °C"));
        h.add(round1(humidityRH));   h.add(F(" %RH"));
      } else {
        t.add(sensorStatusText());
        h.add(sensorStatusText());
      }

      JsonArray usb = user.createNestedArray(F("USB Adalight"));
      usb.add(usbStatusText());

      // machine-readable copy for scripts / HA REST sensors
      JsonObject d = root.createNestedObject(F("wled_assitant"));
      JsonObject s = d.createNestedObject(F("htu21d"));
      s[F("status")] = sensorStatusText();
      if (fresh) {
        s[F("temp_c")]  = round1(temperatureC);
        s[F("rh_pct")]  = round1(humidityRH);
        s[F("age_s")]   = (millis() - lastReadingMs) / 1000;
      }
      s[F("errors")] = errorCount;
      JsonObject u = d.createNestedObject(F("usb"));
      u[F("adalight_rx")] = serialCanRX;
      u[F("streaming")]   = realtimeMode == REALTIME_MODE_ADALIGHT;
      u[F("rx_buffer")]   = WLEDA_USB_RX_BUFFER;
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[F("htuEnabled")]      = htuEnabled;
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
      complete &= getJsonValue(top[F("htuEnabled")],  htuEnabled,  true);
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
        if (!htuEnabled) {
          htuState   = HtuState::Idle;
          htuStatus  = HtuStatus::NotRead;
          htuPresent = false;
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
    bool     htuEnabled  = true;
    uint16_t intervalSec = 30;
    float    tempOffset  = 0.0f;
    float    humOffset   = 0.0f;
    bool     mqttPublish = true;
    bool     haDiscovery = true;
    char     discoveryPrefix[33] = "homeassistant";

    bool     initDone    = false;

    // ---- HTU21D state ----
    enum class HtuState : uint8_t { Idle, Resetting, MeasTemp, MeasHum };
    enum class HtuStatus : uint8_t { NotRead, NoBus, NotFound, ReadError, Ok };
    HtuState  htuState   = HtuState::Idle;
    HtuStatus htuStatus  = HtuStatus::NotRead;
    bool      htuPresent = false;
    uint8_t   polls      = 0;
    uint8_t   consecutiveFails = 0;
    uint32_t  errorCount = 0;
    uint32_t  deadline   = 0;
    uint32_t  nextPollAt = 0;
    uint16_t  rawTemp    = 0;
    uint32_t  lastReadingMs = 0;
    float     temperatureC = NAN;
    float     humidityRH   = NAN;

    volatile bool discoveryDirty = false;   // set from the MQTT task
    bool lastPublishedPresent = false;

    const __FlashStringHelper* usbStatusText() const {
      if (!serialCanRX) return F("disabled (GPIO44/RX is allocated)");
      if (realtimeMode == REALTIME_MODE_ADALIGHT) return F("streaming");
      return F("idle");
    }

    // ------------------------------------------------------------------------------------------
    // HTU21D — one short I2C transaction per step; conversions are waited out across loop() calls
    // ------------------------------------------------------------------------------------------
    bool sendCommand(uint8_t cmd) {
      Wire.beginTransmission(HTU21D_ADDR);
      Wire.write(cmd);
      return Wire.endTransmission() == 0;
    }

    // Returns 1 on success, 0 while the sensor NACKs (conversion not finished), -1 on bad data.
    int8_t readMeasurement(uint16_t& raw) {
      const uint8_t n = Wire.requestFrom(HTU21D_ADDR, uint8_t(3));
      if (n != 3) {
        while (Wire.available()) Wire.read();
        return 0;
      }
      const uint8_t msb = Wire.read(), lsb = Wire.read(), crc = Wire.read();
      if (htuCrc8(msb, lsb) != crc) return -1;
      raw = (uint16_t(msb) << 8 | lsb) & 0xFFFC;           // low 2 bits are status, not data
      return 1;
    }

    void htuFail(uint32_t now, HtuStatus why) {
      errorCount++;
      htuState = HtuState::Idle;
      if (why == HtuStatus::NotFound || ++consecutiveFails >= FAILS_BEFORE_MISSING) {
        htuPresent = false;
        htuStatus  = why;
        nextPollAt = now + uint32_t(intervalSec) * 1000;
      } else {
        nextPollAt = now + RETRY_AFTER_ERROR_MS;
      }
      publishAvailabilityIfChanged();
    }

    void startTemperature(uint32_t now) {
      if (!sendCommand(HTU21D_TRIG_T_NOHOLD)) { htuFail(now, HtuStatus::NotFound); return; }
      htuState = HtuState::MeasTemp;
      deadline = now + HTU21D_T_CONV_MS;
      polls = 0;
    }

    void serviceHtu(uint32_t now) {
      if (i2c_sda < 0 || i2c_scl < 0) { htuStatus = HtuStatus::NoBus; return; }

      switch (htuState) {
        case HtuState::Idle:
          if (!timeReached(now, nextPollAt)) return;
          if (!htuPresent) {
            if (!sendCommand(HTU21D_SOFT_RESET)) { htuFail(now, HtuStatus::NotFound); return; }
            htuState = HtuState::Resetting;
            deadline = now + HTU21D_RESET_MS;
            return;
          }
          startTemperature(now);
          return;

        case HtuState::Resetting:
          if (!timeReached(now, deadline)) return;
          startTemperature(now);
          return;

        case HtuState::MeasTemp: {
          if (!timeReached(now, deadline)) return;
          const int8_t r = readMeasurement(rawTemp);
          if (r == 0 && ++polls < HTU21D_MAX_POLLS) { deadline = now + HTU21D_POLL_MS; return; }
          if (r != 1) { htuFail(now, HtuStatus::ReadError); return; }
          if (!sendCommand(HTU21D_TRIG_H_NOHOLD)) { htuFail(now, HtuStatus::ReadError); return; }
          htuState = HtuState::MeasHum;
          deadline = now + HTU21D_H_CONV_MS;
          polls = 0;
          return;
        }

        case HtuState::MeasHum: {
          if (!timeReached(now, deadline)) return;
          uint16_t rawHum = 0;
          const int8_t r = readMeasurement(rawHum);
          if (r == 0 && ++polls < HTU21D_MAX_POLLS) { deadline = now + HTU21D_POLL_MS; return; }
          if (r != 1) { htuFail(now, HtuStatus::ReadError); return; }

          temperatureC = -46.85f + 175.72f * rawTemp / 65536.0f + tempOffset;
          humidityRH   = constrain(-6.0f + 125.0f * rawHum / 65536.0f + humOffset, 0.0f, 100.0f);
          lastReadingMs = now;
          htuPresent = true;
          htuStatus  = HtuStatus::Ok;
          consecutiveFails = 0;
          htuState   = HtuState::Idle;
          nextPollAt = now + uint32_t(intervalSec) * 1000;
          publishAvailabilityIfChanged();
          publishReadings();
          return;
        }
      }
    }

    bool readingFresh(uint32_t now) const {
      return lastReadingMs != 0 && htuStatus == HtuStatus::Ok &&
             (now - lastReadingMs) < 3UL * intervalSec * 1000;
    }

    const __FlashStringHelper* sensorStatusText() const {
      if (!htuEnabled) return F("disabled");
      switch (htuStatus) {
        case HtuStatus::NoBus:     return F("no I2C bus - set SDA/SCL in Usermods settings");
        case HtuStatus::NotFound:  return F("HTU21D not found at 0x40");
        case HtuStatus::ReadError: return F("read error (CRC/NACK)");
        case HtuStatus::Ok:        return F("ok");
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
      snprintf_P(buf, len, PSTR("%s/htu21d/%s"), mqttDeviceTopic, leaf);
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
        "\"stat_t\":\"%s/htu21d/%s\",\"unit_of_meas\":\"%s\",\"dev_cla\":\"%s\",\"stat_cla\":\"measurement\","
        "\"sug_dsp_prec\":1,\"avty_mode\":\"all\",\"avty\":[{\"t\":\"%s/status\"},{\"t\":\"%s/htu21d/status\"}],"
        "\"dev\":{\"cns\":[[\"mac\",\"%.2s:%.2s:%.2s:%.2s:%.2s:%.2s\"]],\"name\":\"%s\",\"mf\":\"WLED\","
        "\"mdl\":\"ESP32-S3 SuperMini + HTU21D\",\"sw\":\"%s\"}}"),
        name, m, key, m, key,
        mqttDeviceTopic, key, unit, devClass,
        mqttDeviceTopic, mqttDeviceTopic,
        m, m + 2, m + 4, m + 6, m + 8, m + 10, devName,
        versionString);
      mqtt->publish(topic, 0, true, payload);
    }

    void publishDiscovery() {
      if (!WLED_MQTT_CONNECTED || !mqttDeviceTopic[0] || escapedMac.length() < 12) return;
      publishDiscoveryEntity("temperature", "Temperature", "°C", "temperature");
      publishDiscoveryEntity("humidity", "Humidity", "%", "humidity");
    }

    void publishAvailability() {
      if (!mqttReady()) return;
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      topicFor(topic, sizeof(topic), "status");
      mqtt->publish(topic, 0, true, htuPresent ? "online" : "offline");
      lastPublishedPresent = htuPresent;
    }

    void publishAvailabilityIfChanged() {
      if (htuPresent != lastPublishedPresent) publishAvailability();
    }

    void publishReadings() {
      if (!mqttReady() || !readingFresh(millis())) return;
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      char value[16];
      topicFor(topic, sizeof(topic), "temperature");
      snprintf_P(value, sizeof(value), PSTR("%.1f"), temperatureC);
      mqtt->publish(topic, 0, true, value);
      topicFor(topic, sizeof(topic), "humidity");
      snprintf_P(value, sizeof(value), PSTR("%.1f"), humidityRH);
      mqtt->publish(topic, 0, true, value);
    }
#else
    void publishAvailabilityIfChanged() {}
    void publishReadings() {}
#endif
};

const char WledAssitant::_name[] PROGMEM = "wled_assitant";

static WledAssitant wled_assitant;
REGISTER_USERMOD(wled_assitant);
