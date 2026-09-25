/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - TXS0108E OE lifecycle (translator held off until WLED's LED output is initialised and idle)
 *  - HTU21D temperature / humidity over WLED's global I2C bus (non-blocking state machine)
 *  - Info/JSON diagnostics, including the USB Adalight path
 *
 * The HyperHDR stream itself is handled by stock WLED (wled00/wled_serial.cpp, Adalight parser
 * reading `Serial`, which is the ESP32-S3 USB-Serial/JTAG "HWCDC" port with the build flags in
 * platformio_override.ini). This usermod only sizes that port's RX queue; it never renders LEDs.
 *
 * All pins come from build flags generated from the [wled_assitant] section of
 * platformio_override.ini — do not hard-code GPIO numbers here.
 */
#include "wled.h"
#include "driver/gpio.h"
#include "esp_system.h"

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

#if !defined(WLEDA_LED_DATA_PIN) || !defined(WLEDA_TXS_OE_PIN) || !defined(I2CSDAPIN) || !defined(I2CSCLPIN)
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
static_assert(s3PinUsable(WLEDA_TXS_OE_PIN),   "TXS0108E OE pin is reserved/boot-critical on ESP32-S3");
static_assert(s3PinUsable(I2CSDAPIN),          "I2C SDA pin is reserved/boot-critical on ESP32-S3");
static_assert(s3PinUsable(I2CSCLPIN),          "I2C SCL pin is reserved/boot-critical on ESP32-S3");
static_assert(WLEDA_LED_DATA_PIN != WLEDA_TXS_OE_PIN && WLEDA_LED_DATA_PIN != I2CSDAPIN && WLEDA_LED_DATA_PIN != I2CSCLPIN &&
              WLEDA_TXS_OE_PIN != I2CSDAPIN && WLEDA_TXS_OE_PIN != I2CSCLPIN && I2CSDAPIN != I2CSCLPIN,
              "LED data, OE, SDA and SCL pins must all be different");

constexpr gpio_num_t OE_GPIO = static_cast<gpio_num_t>(WLEDA_TXS_OE_PIN);

// Raw IDF calls: these are used before Arduino/WLED are up and from the restart path.
// Level is latched before the output driver is enabled, so the pin never glitches high.
void oeForceLow() {
  gpio_set_level(OE_GPIO, 0);
  gpio_config_t cfg = {};
  cfg.pin_bit_mask  = 1ULL << WLEDA_TXS_OE_PIN;
  cfg.mode          = GPIO_MODE_OUTPUT;
  cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
  cfg.pull_down_en  = GPIO_PULLDOWN_ENABLE;   // backs up the external 10k pull-down
  cfg.intr_type     = GPIO_INTR_DISABLE;
  gpio_config(&cfg);
  gpio_set_level(OE_GPIO, 0);
}

// Runs from esp_restart() (WLED reboot, OTA completion). Panics/brown-outs do not run it; there the
// reset releases the pin and the external pull-down turns the translator off.
void oeShutdownHandler() { gpio_set_level(OE_GPIO, 0); }

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
      oeForceLow();
      esp_register_shutdown_handler(oeShutdownHandler);
      // Stock HWCDC RX queue is 256 B — smaller than one Adalight frame — and the HWCDC ISR drops
      // bytes when it is full. HWCDC::begin() (called later by WLED) keeps a pre-sized queue, so
      // resizing here avoids replacing the queue while the USB ISR is live.
      Serial.setRxBufferSize(WLEDA_USB_RX_BUFFER);
    }

    void setup() override {
      oeForceLow();
      oeAllocated = PinManager::allocatePin(WLEDA_TXS_OE_PIN, true, PinOwner::UM_Unspecified);
      if (!oeAllocated) oeForceLow();   // someone else owns it; we cannot drive it high safely
      setupMs = millis();

      if (i2c_sda >= 0 && i2c_scl >= 0) Wire.setTimeOut(WLEDA_I2C_TIMEOUT_MS);
      nextPollAt = setupMs + 2000;      // first reading shortly after boot, off the critical path
      initDone = true;
    }

    void loop() override {
      const uint32_t now = millis();
      serviceOe(now);
      if (htuEnabled) serviceHtu(now);
    }

    void onUpdateBegin(bool init) override {
      // Keep the strip isolated while flash is being written; re-enabled by serviceOe() on failure.
      oeSuspended = init;
      if (init) setOe(false);
    }

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

      JsonArray oe = user.createNestedArray(F("LED level shifter"));
      oe.add(oeStatusText());

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
      JsonObject o = d.createNestedObject(F("oe"));
      o[F("gpio")]    = WLEDA_TXS_OE_PIN;
      o[F("enabled")] = oeEnabled;
      if (oeEnabled) o[F("enabled_at_ms")] = oeEnabledAt;
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
      top[F("oeDelayMs")]       = oeDelayMs;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool complete = !top.isNull();
      complete &= getJsonValue(top[F("htuEnabled")],  htuEnabled,  true);
      complete &= getJsonValue(top[F("interval")],    intervalSec, uint16_t(30));
      complete &= getJsonValue(top[F("tempOffset")],  tempOffset,  0.0f);
      complete &= getJsonValue(top[F("humOffset")],   humOffset,   0.0f);
      complete &= getJsonValue(top[F("oeDelayMs")],   oeDelayMs,   uint16_t(0));

      intervalSec = constrain(intervalSec, 5, 3600);
      if (oeDelayMs > 5000) oeDelayMs = 5000;

      if (initDone) {
        if (!htuEnabled) {
          htuState   = HtuState::Idle;
          htuStatus  = HtuStatus::NotRead;
          htuPresent = false;
        }
        nextPollAt = millis();          // apply new settings with a fresh reading
      }
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("addInfo('wled_assitant:interval',1,'s (5-3600)');"));
      s.print(F("addInfo('wled_assitant:tempOffset',1,'°C');"));
      s.print(F("addInfo('wled_assitant:humOffset',1,'%RH');"));
      s.print(F("addInfo('wled_assitant:oeDelayMs',1,'ms extra hold before OE goes high (GPIO"));
      s.print(WLEDA_TXS_OE_PIN);
      s.print(F(", fixed at build time)');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  private:
    static const char _name[];

    // ---- settings (cfg.json "um" → "wled_assitant") ----
    bool     htuEnabled  = true;
    uint16_t intervalSec = 30;
    float    tempOffset  = 0.0f;
    float    humOffset   = 0.0f;
    uint16_t oeDelayMs   = 0;

    // ---- OE state ----
    bool     initDone    = false;
    bool     oeAllocated = false;
    bool     oeEnabled   = false;
    bool     oeSuspended = false;
    uint32_t setupMs     = 0;
    uint32_t oeEnabledAt = 0;

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

    // ------------------------------------------------------------------------------------------
    // TXS0108E OE
    // ------------------------------------------------------------------------------------------
    void setOe(bool on) {
      gpio_set_level(OE_GPIO, on ? 1 : 0);
      oeEnabled = on;
      if (on) oeEnabledAt = millis();
    }

    // The translator is enabled only once WLED's digital LED bus owns the data GPIO (i.e. the RMT
    // driver is configured and idling LOW) and no frame is in flight, so the strip never sees a
    // partial frame. If the LED output is later removed/moved, the translator is switched off again.
    void serviceOe(uint32_t now) {
      if (!oeAllocated || oeSuspended) return;
      const bool dataPinReady = PinManager::getPinOwner(WLEDA_LED_DATA_PIN) == PinOwner::BusDigital;
      if (oeEnabled) {
        if (!dataPinReady) setOe(false);
        return;
      }
      if (!dataPinReady || (now - setupMs) < oeDelayMs || strip.isUpdating()) return;
      setOe(true);
      strip.trigger();   // DIN floated while OE was low: repaint so any latched noise is overwritten
    }

    const __FlashStringHelper* oeStatusText() const {
      if (!oeAllocated) return F("OFF - OE GPIO already in use (pin conflict)");
      if (oeSuspended)  return F("OFF - firmware update in progress");
      if (oeEnabled)    return F("enabled");
      if (PinManager::getPinOwner(WLEDA_LED_DATA_PIN) != PinOwner::BusDigital)
        return F("OFF - LED data GPIO is not a WLED LED output");
      return F("OFF - waiting for LED driver");
    }

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
};

const char WledAssitant::_name[] PROGMEM = "wled_assitant";

static WledAssitant wled_assitant;
REGISTER_USERMOD(wled_assitant);
