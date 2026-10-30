/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - BMP280 temperature / pressure over WLED's global I2C bus (non-blocking state machine)
 *  - DHT11 humidity on its own GPIO, captured by an RMT receive channel (non-blocking)
 *  - dew point from BMP280 temperature + DHT11 humidity
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
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
#include "soc/gpio_sig_map.h"

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

#if !defined(WLEDA_LED_DATA_PIN) || !defined(I2CSDAPIN) || !defined(I2CSCLPIN) || !defined(WLEDA_DHT_PIN)
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
static_assert(s3PinUsable(WLEDA_DHT_PIN),      "DHT11 data pin is reserved/boot-critical on ESP32-S3");
static_assert(WLEDA_LED_DATA_PIN != I2CSDAPIN && WLEDA_LED_DATA_PIN != I2CSCLPIN && I2CSDAPIN != I2CSCLPIN &&
              WLEDA_DHT_PIN != WLEDA_LED_DATA_PIN && WLEDA_DHT_PIN != I2CSDAPIN && WLEDA_DHT_PIN != I2CSCLPIN,
              "LED data, SDA, SCL and DHT11 pins must all be different");

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

// DHT11 (Aosong datasheet): the host holds the line low for 18-30 ms, then the sensor answers
// with 80 us low + 80 us high and 40 bits, each 50 us low + 26-28 us ("0") or 70 us ("1") high.
constexpr uint32_t DHT_START_LOW_MS   = 20;
constexpr uint32_t DHT_FRAME_TIMEOUT_MS = 20;            // a frame takes ~5 ms
constexpr uint32_t DHT_FIRST_READ_MS  = 2500;            // sensor needs 1 s after power-up
constexpr uint16_t DHT_IDLE_US        = 1000;            // capture ends 1 ms after the last edge
constexpr uint16_t DHT_BIT1_MIN_US    = 48;
constexpr uint16_t DHT_PULSE_MAX_US   = 120;
constexpr uint8_t  DHT_FRAME_BITS     = 40;
// LL RX index 0 = RMT channel 4. S3 channels 4-7 only receive, so WLED's LED buses (TX 0-3) never
// take it; its memory is block 4 of RMTMEM.
constexpr uint32_t DHT_RMT_RX         = 0;
constexpr uint32_t DHT_RMT_MEM_BLOCK  = SOC_RMT_TX_CANDIDATES_PER_GROUP + DHT_RMT_RX;
constexpr uint32_t DHT_RMT_WORDS      = SOC_RMT_MEM_WORDS_PER_CHANNEL;
constexpr uint32_t DHT_RMT_END_BIT    = 1u << (16 + DHT_RMT_RX);   // int_raw: ch4 rx_end
constexpr uint32_t DHT_RMT_ERR_BIT    = 1u << (20 + DHT_RMT_RX);   // int_raw: ch4 rx_err (memory full)

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

// words = raw RMT items (two level/duration halves each, duration 0 = end), 1 us per tick.
// The data bits are the last 40 high pulses that end in a low; whatever precedes them (the
// release after our start pulse, the sensor's 80 us response) is skipped, and a trailing idle
// high never ends in a low. Returns nullptr on success, else a short reason.
const char* dhtDecode(const uint32_t* words, size_t n, uint8_t out[5]) {
  uint16_t highs[2 * DHT_RMT_WORDS];
  size_t nh = 0;
  uint16_t pendingHigh = 0;
  for (size_t i = 0; i < 2 * n; i++) {
    const uint16_t half = (i & 1) ? uint16_t(words[i / 2] >> 16) : uint16_t(words[i / 2]);
    const uint16_t dur = half & 0x7FFF;
    if (dur == 0) break;
    if (half & 0x8000)    pendingHigh = dur;
    else if (pendingHigh) { highs[nh++] = pendingHigh; pendingHigh = 0; }
  }
  if (nh == 0) return "no response";
  if (nh < DHT_FRAME_BITS + 1) return "short frame";

  const uint16_t* bits = highs + nh - DHT_FRAME_BITS;
  const uint16_t response = bits[-1];
  if (response < DHT_BIT1_MIN_US || response > DHT_PULSE_MAX_US) return "bad response";
  memset(out, 0, 5);
  for (uint8_t i = 0; i < DHT_FRAME_BITS; i++) {
    if (bits[i] > DHT_PULSE_MAX_US) return "bad pulse";
    out[i / 8] = uint8_t(out[i / 8] << 1 | (bits[i] >= DHT_BIT1_MIN_US));
  }
  if (uint8_t(out[0] + out[1] + out[2] + out[3]) != out[4]) return "checksum";
  if (!(out[0] | out[1] | out[2] | out[3])) return "all zero";   // passes the checksum, never a real reading
  return nullptr;
}

// DHT11 frame: RH integer, RH tenths, T integer, T tenths (bit 7 = below 0 °C), checksum.
// A DHT22 sends 16-bit values instead, which shows up as "tenths" above 9.
const char* dhtParse(const uint8_t b[5], int16_t& rhTenths, int16_t& tTenths) {
  if (b[1] > 9 || (b[3] & 0x7F) > 9) return "not a DHT11";
  rhTenths = int16_t(b[0] * 10 + b[1]);
  tTenths  = int16_t(b[2] * 10 + (b[3] & 0x7F));
  if (b[3] & 0x80) tTenths = int16_t(-tTenths);
  if (rhTenths > 1000 || tTenths < -200 || tTenths > 600) return "out of range";
  return nullptr;
}

// Magnus formula (Sonntag 1990 constants); valid for -45..60 °C over water.
float dewPointC(float tC, float rhPct) {
  const float g = logf(rhPct / 100.0f) + 17.62f * tC / (243.12f + tC);
  return 243.12f * g / (17.62f - g);
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
      dhtNextPollAt = millis() + DHT_FIRST_READ_MS;
      dhtSetup();
      initDone = true;
    }

    void loop() override {
      const uint32_t now = millis();
      if (bmpEnabled) serviceBmp(now);
      if (dhtEnabled) serviceDht(now);
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

      const uint32_t now = millis();
      const bool bmpFresh = bmpReadingFresh(now);
      const bool dhtFresh = dhtReadingFresh(now);
      JsonArray t  = user.createNestedArray(F("Temperature"));
      JsonArray h  = user.createNestedArray(F("Humidity"));
      JsonArray p  = user.createNestedArray(F("Pressure"));
      JsonArray dp = user.createNestedArray(F("Dew point"));
      if (bmpFresh) {
        t.add(round1(temperatureC)); t.add(F(" °C"));
        p.add(round1(pressureHPa));  p.add(F(" hPa"));
      } else {
        t.add(bmpStatusText());
        p.add(bmpStatusText());
      }
      if (dhtFresh) { h.add(round1(humidityPct)); h.add(F(" %")); }
      else            h.add(dhtStatusText());
      if (bmpFresh && dhtFresh) { dp.add(round1(dewPoint())); dp.add(F(" °C")); }
      else                        dp.add(F("needs BMP280 + DHT11"));

      JsonArray usb = user.createNestedArray(F("USB Adalight"));
      usb.add(usbStatusText());

      // machine-readable copy for scripts / HA REST sensors
      JsonObject d = root.createNestedObject(F("wled_assitant"));
      JsonObject s = d.createNestedObject(F("bmp280"));
      s[F("status")] = bmpStatusText();
      if (bmpAddr) s[F("addr")] = bmpAddr;
      if (bmpFresh) {
        s[F("temp_c")] = round1(temperatureC);
        s[F("hpa")]    = round1(pressureHPa);
        s[F("age_s")]  = (now - lastReadingMs) / 1000;
        s[F("raw_t")]  = rawTemp;
        s[F("raw_p")]  = rawPress;
      }
      s[F("errors")] = errorCount;
      if (lastFault[0]) s[F("last_fault")] = lastFault;

      JsonObject dh = d.createNestedObject(F("dht11"));
      dh[F("status")] = dhtStatusText();
      dh[F("pin")]    = WLEDA_DHT_PIN;
      if (dhtFresh) {
        dh[F("rh_pct")] = round1(humidityPct);
        dh[F("temp_c")] = dhtTempC;     // the DHT11's own reading (±2 °C); BMP280 is the temperature
        dh[F("age_s")]  = (now - dhtLastReadingMs) / 1000;
        char raw[11];
        snprintf_P(raw, sizeof(raw), PSTR("%02X%02X%02X%02X%02X"), dhtRaw[0], dhtRaw[1], dhtRaw[2], dhtRaw[3], dhtRaw[4]);
        dh[F("raw")] = raw;
      }
      dh[F("errors")] = dhtErrorCount;
      if (dhtFault[0]) dh[F("last_fault")] = dhtFault;
      if (bmpFresh && dhtFresh) d[F("dew_point_c")] = round1(dewPoint());

      JsonObject u = d.createNestedObject(F("usb"));
      u[F("adalight_rx")] = serialCanRX;
      u[F("streaming")]   = realtimeMode == REALTIME_MODE_ADALIGHT;
      u[F("rx_buffer")]   = WLEDA_USB_RX_BUFFER;
    }

    void addToConfig(JsonObject& root) override {
      JsonObject top = root.createNestedObject(FPSTR(_name));
      top[F("bmpEnabled")]      = bmpEnabled;
      top[F("dhtEnabled")]      = dhtEnabled;
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
      complete &= getJsonValue(top[F("bmpEnabled")],  bmpEnabled,  true);
      complete &= getJsonValue(top[F("dhtEnabled")],  dhtEnabled,  true);
      complete &= getJsonValue(top[F("interval")],    intervalSec, uint16_t(30));
      complete &= getJsonValue(top[F("tempOffset")],  tempOffset,  0.0f);
      complete &= getJsonValue(top[F("humOffset")],   humOffset,   0.0f);
      complete &= getJsonValue(top[F("mqttPublish")], mqttPublish, true);
      complete &= getJsonValue(top[F("haDiscovery")], haDiscovery, true);
      const char* prefix = top[F("discoveryPrefix")] | "homeassistant";
      strlcpy(discoveryPrefix, prefix[0] ? prefix : "homeassistant", sizeof(discoveryPrefix));
      complete &= !top[F("discoveryPrefix")].isNull();

      intervalSec = constrain(intervalSec, 5, 3600);
      humOffset   = constrain(humOffset, -50.0f, 50.0f);

      if (initDone) {
        if (!bmpEnabled) {
          bmpState   = BmpState::Idle;
          bmpStatus  = BmpStatus::NotRead;
          bmpPresent = false;
        }
        if (!dhtEnabled && dhtReady) {     // keep PinBusy / RmtError visible
          dhtAbortFrame();
          dhtStatus  = DhtStatus::NotRead;
          dhtPresent = false;
        }
        nextPollAt = millis();          // apply new settings with a fresh reading
        dhtNextPollAt = millis();
        discoveryDirty = true;          // republish (or clear) HA discovery
      }
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("addInfo('wled_assitant:interval',1,'s (5-3600)');"));
      s.print(F("addInfo('wled_assitant:tempOffset',1,'°C');"));
      s.print(F("addInfo('wled_assitant:humOffset',1,'%RH');"));
      s.print(F("addInfo('wled_assitant:dhtEnabled',1,'DHT11 on GPIO"));
      s.print(WLEDA_DHT_PIN);
      s.print(F("');"));
      s.print(F("addInfo('wled_assitant:mqttPublish',1,'uses the MQTT settings in Sync Interfaces; no broker is fine');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  private:
    static const char _name[];

    // ---- settings (cfg.json "um" → "wled_assitant") ----
    bool     bmpEnabled  = true;
    bool     dhtEnabled  = true;
    uint16_t intervalSec = 30;
    float    tempOffset  = 0.0f;
    float    humOffset   = 0.0f;
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

    // ---- DHT11 state ----
    enum class DhtState : uint8_t { Idle, StartLow, Receiving };
    enum class DhtStatus : uint8_t { NotRead, PinBusy, RmtError, NoResponse, ReadError, Ok };
    DhtState  dhtState   = DhtState::Idle;
    DhtStatus dhtStatus  = DhtStatus::NotRead;
    bool      dhtReady   = false;  // pin allocated and RMT channel configured
    bool      dhtPresent = false;
    uint8_t   dhtConsecutiveFails = 0;
    uint32_t  dhtErrorCount = 0;
    uint32_t  dhtDeadline   = 0;
    uint32_t  dhtNextPollAt = 0;
    uint32_t  dhtLastReadingMs = 0;
    uint8_t   dhtRaw[5] = {};
    char      dhtFault[24] = "";
    float     humidityPct = NAN;
    float     dhtTempC    = NAN;

    volatile bool discoveryDirty = false;   // set from the MQTT task
    bool lastPublishedBmp = false;
    bool lastPublishedDht = false;

    const __FlashStringHelper* usbStatusText() const {
      if (!serialCanRX) return F("disabled (GPIO44/RX is allocated)");
      if (realtimeMode == REALTIME_MODE_ADALIGHT) return F("streaming");
      return F("idle");
    }

    float dewPoint() const { return dewPointC(temperatureC, max(humidityPct, 1.0f)); }

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
      publishBmpReadings();
    }

    bool bmpReadingFresh(uint32_t now) const {
      return lastReadingMs != 0 && bmpStatus == BmpStatus::Ok &&
             (now - lastReadingMs) < 3UL * intervalSec * 1000;
    }

    const __FlashStringHelper* bmpStatusText() const {
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
    // DHT11 — the 20 ms start pulse is timed across loop() calls and the reply is captured by an
    // RMT receive channel, so nothing blocks and no interrupts are masked. Bit-banging would need
    // ~4 ms with interrupts off, which starves the LED driver's RMT refill interrupt.
    // The channel is driven through the RMT registers only: WLED's LED driver
    // (lib/NeoESP32RmtHI) owns the RMT interrupt exclusively, so the IDF RMT RX driver (which
    // installs its own ISR) can't be used. Completion is polled from int_raw; our interrupt
    // enable bits stay off.
    // ------------------------------------------------------------------------------------------
    void dhtSetup() {
      if (!PinManager::allocatePin(WLEDA_DHT_PIN, true, PinOwner::UM_Unspecified)) {
        dhtStatus = DhtStatus::PinBusy;
        return;
      }
      gpio_set_level(gpio_num_t(WLEDA_DHT_PIN), 1);          // released; the module's pull-up holds it high
      gpio_config_t io = {};
      io.pin_bit_mask = 1ULL << WLEDA_DHT_PIN;
      io.mode         = GPIO_MODE_INPUT_OUTPUT_OD;
      io.pull_up_en   = GPIO_PULLUP_ENABLE;
      if (gpio_config(&io) != ESP_OK || !dhtRmtConfigure()) {
        dhtStatus = DhtStatus::RmtError;
        return;
      }
      esp_rom_gpio_connect_in_signal(WLEDA_DHT_PIN, RMT_SIG_IN0_IDX + DHT_RMT_RX, false);
      dhtReady = true;
    }

    // The group clock is shared with the LED channels: read it, and only set it up when nothing
    // has (LED bus on I2S, RMT never started). Rewriting it would glitch a running LED frame.
    bool dhtRmtConfigure() {
      periph_module_enable(PERIPH_RMT_MODULE);               // reference-counted; no reset if already on
      if (!RMT.sys_conf.clk_en) {
        rmt_ll_enable_drive_clock(&RMT, true);
        rmt_ll_power_down_mem(&RMT, false);
        rmt_ll_set_group_clock_src(&RMT, 0, 1 /* APB */, 0, 0, 0);
      }
      rmt_ll_enable_mem_access(&RMT, true);                  // direct RMTMEM access, as the LED driver uses

      const uint32_t src = RMT.sys_conf.sclk_sel == 1 ? APB_CLK_FREQ : RMT.sys_conf.sclk_sel == 3 ? 40000000 : 0;
      if (src == 0 || RMT.sys_conf.sclk_div_a != 0) {
        strcpy_P(dhtFault, PSTR("rmt clock unknown"));
        return false;
      }
      const uint32_t ticksPerUs = src / (1 + RMT.sys_conf.sclk_div_num) / 1000000;
      if (ticksPerUs < 1 || ticksPerUs > 255) {
        snprintf_P(dhtFault, sizeof(dhtFault), PSTR("rmt clock %lu MHz"), (unsigned long)ticksPerUs);
        return false;
      }

      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, false);
      rmt_ll_rx_set_channel_clock_div(&RMT, DHT_RMT_RX, ticksPerUs);     // 1 us per item tick
      rmt_ll_rx_set_mem_blocks(&RMT, DHT_RMT_RX, 1);                       // 48 words; a frame needs ~43
      rmt_ll_rx_set_idle_thres(&RMT, DHT_RMT_RX, DHT_IDLE_US);
      rmt_ll_rx_enable_filter(&RMT, DHT_RMT_RX, true);
      rmt_ll_rx_set_filter_thres(&RMT, DHT_RMT_RX, std::min<uint32_t>(2 * ticksPerUs, 255));   // ignore glitches < 2 us
      rmt_ll_rx_enable_carrier_demodulation(&RMT, DHT_RMT_RX, false);     // on by default
      rmt_ll_rx_enable_pingpong(&RMT, DHT_RMT_RX, false);
      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, false);                           // latches the config (conf_update)
      return true;
    }

    void dhtAbortFrame() {
      if (!dhtReady) return;
      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, false);
      gpio_set_level(gpio_num_t(WLEDA_DHT_PIN), 1);
      dhtState = DhtState::Idle;
    }

    void serviceDht(uint32_t now) {
      if (!dhtReady) return;
      switch (dhtState) {
        case DhtState::Idle:
          if (!timeReached(now, dhtNextPollAt)) return;
          gpio_set_level(gpio_num_t(WLEDA_DHT_PIN), 0);       // start pulse
          dhtState    = DhtState::StartLow;
          dhtDeadline = now + DHT_START_LOW_MS;
          return;
        case DhtState::StartLow:  dhtRelease(now); return;
        case DhtState::Receiving: dhtCollect(now); return;
      }
    }

    void dhtRelease(uint32_t now) {
      if (!timeReached(now, dhtDeadline)) return;
      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, false);
      rmt_ll_rx_reset_pointer(&RMT, DHT_RMT_RX);
      rmt_ll_rx_set_mem_owner(&RMT, DHT_RMT_RX, RMT_MEM_OWNER_RX);
      RMT.int_clr.val = DHT_RMT_END_BIT | DHT_RMT_ERR_BIT;
      // Capture must already run when the line is released; a preemption between the two steps
      // longer than the idle threshold would end the capture before the sensor answers.
      static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
      portENTER_CRITICAL(&mux);
      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, true);
      gpio_set_level(gpio_num_t(WLEDA_DHT_PIN), 1);
      portEXIT_CRITICAL(&mux);
      dhtState    = DhtState::Receiving;
      dhtDeadline = now + DHT_FRAME_TIMEOUT_MS;
    }

    void dhtCollect(uint32_t now) {
      const uint32_t raw = RMT.int_raw.val;
      if (!(raw & (DHT_RMT_END_BIT | DHT_RMT_ERR_BIT))) {
        if (!timeReached(now, dhtDeadline)) return;
        dhtAbortFrame();
        dhtFail(now, DhtStatus::NoResponse, PSTR("capture timeout"));
        return;
      }
      rmt_ll_rx_enable(&RMT, DHT_RMT_RX, false);
      RMT.int_clr.val = DHT_RMT_END_BIT | DHT_RMT_ERR_BIT;
      if (raw & DHT_RMT_ERR_BIT) {                            // more edges than fit: noise on the line
        dhtState = DhtState::Idle;
        dhtFail(now, DhtStatus::ReadError, PSTR("rmt overflow"));
        return;
      }

      uint32_t words[DHT_RMT_WORDS];
      rmt_ll_rx_set_mem_owner(&RMT, DHT_RMT_RX, RMT_MEM_OWNER_TX);   // hand the block to software
      for (uint32_t i = 0; i < DHT_RMT_WORDS; i++) words[i] = RMTMEM.chan[DHT_RMT_MEM_BLOCK].data32[i].val;
      rmt_ll_rx_set_mem_owner(&RMT, DHT_RMT_RX, RMT_MEM_OWNER_RX);
      dhtState = DhtState::Idle;

      uint8_t b[5];
      const char* why = dhtDecode(words, DHT_RMT_WORDS, b);
      if (why) {
        dhtFail(now, strcmp(why, "no response") ? DhtStatus::ReadError : DhtStatus::NoResponse, why);
        return;
      }
      memcpy(dhtRaw, b, sizeof(dhtRaw));
      int16_t rhTenths, tTenths;
      if ((why = dhtParse(b, rhTenths, tTenths))) {
        snprintf_P(dhtFault, sizeof(dhtFault), PSTR("%s %02X%02X%02X%02X"), why, b[0], b[1], b[2], b[3]);
        dhtFail(now, DhtStatus::ReadError, nullptr);
        return;
      }

      humidityPct = constrain(rhTenths / 10.0f + humOffset, 0.0f, 100.0f);
      dhtTempC    = tTenths / 10.0f;
      dhtLastReadingMs = now;
      dhtPresent = true;
      dhtStatus  = DhtStatus::Ok;
      dhtConsecutiveFails = 0;
      dhtNextPollAt = now + uint32_t(intervalSec) * 1000;
      publishAvailabilityIfChanged();
      publishDhtReadings();
    }

    // why: PROGMEM/RAM reason copied to dhtFault, or nullptr if the caller already set it.
    void dhtFail(uint32_t now, DhtStatus status, const char* why) {
      if (why) strlcpy(dhtFault, why, sizeof(dhtFault));
      dhtErrorCount++;
      if (++dhtConsecutiveFails >= FAILS_BEFORE_MISSING) {
        dhtPresent = false;
        dhtStatus  = status;
        dhtNextPollAt = now + uint32_t(intervalSec) * 1000;
      } else {
        dhtNextPollAt = now + RETRY_AFTER_ERROR_MS;
      }
      publishAvailabilityIfChanged();
    }

    bool dhtReadingFresh(uint32_t now) const {
      return dhtLastReadingMs != 0 && dhtStatus == DhtStatus::Ok &&
             (now - dhtLastReadingMs) < 3UL * intervalSec * 1000;
    }

    const __FlashStringHelper* dhtStatusText() const {
      if (!dhtEnabled) return F("disabled");
      switch (dhtStatus) {
        case DhtStatus::PinBusy:    return F("DHT11 pin is used by something else");
        case DhtStatus::RmtError:   return F("RMT setup failed - see last_fault");
        case DhtStatus::NoResponse: return F("DHT11 not responding");
        case DhtStatus::ReadError:  return F("read error - see last_fault");
        case DhtStatus::Ok:         return F("ok");
        default:                    return F("not read yet");
      }
    }

    // ------------------------------------------------------------------------------------------
    // MQTT / Home Assistant discovery — strictly optional; every call is a no-op without a broker
    // ------------------------------------------------------------------------------------------
    enum : uint8_t { NEEDS_BMP = 1, NEEDS_DHT = 2 };
    struct Entity {
      const char* key; const char* name; const char* unit; const char* devClass;
      const char* sensor;      // state topic <topic>/<sensor>/<key>, or <topic>/<key> if null
      uint8_t needs;
    };
    // Unique IDs wled_<mac>_temperature/_humidity predate this sensor pair, so HA keeps their history.
    static constexpr Entity ENTITIES[] = {
      { "temperature", "Temperature", "°C",  "temperature",          "bmp280", NEEDS_BMP },
      { "pressure",    "Pressure",    "hPa", "atmospheric_pressure", "bmp280", NEEDS_BMP },
      { "humidity",    "Humidity",    "%",   "humidity",             "dht11",  NEEDS_DHT },
      { "dew_point",   "Dew point",   "°C",  "temperature",          nullptr,  NEEDS_BMP | NEEDS_DHT },
    };
    static constexpr uint8_t E_TEMP = 0, E_PRESS = 1, E_HUM = 2, E_DEW = 3;

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

    bool entityEnabled(const Entity& e) const {
      return (!(e.needs & NEEDS_BMP) || bmpEnabled) && (!(e.needs & NEEDS_DHT) || dhtEnabled);
    }

    static void stateTopic(char* buf, size_t len, const Entity& e) {
      if (e.sensor) snprintf_P(buf, len, PSTR("%s/%s/%s"), mqttDeviceTopic, e.sensor, e.key);
      else          snprintf_P(buf, len, PSTR("%s/%s"), mqttDeviceTopic, e.key);
    }

    void publishDiscoveryEntity(const Entity& e) {
      static char topic[128];
      static char payload[1024];   // static: no per-publish heap churn
      snprintf_P(topic, sizeof(topic), PSTR("%s/sensor/wled_%s/%s/config"), discoveryPrefix, escapedMac.c_str(), e.key);

      if (!haDiscovery || !mqttPublish || !entityEnabled(e)) {   // remove previously announced entity
        mqtt->publish(topic, 0, true, "");
        return;
      }

      char state[MQTT_MAX_TOPIC_LEN + 24];
      stateTopic(state, sizeof(state), e);
      // availability: WLED's LWT plus the status topic of every sensor the value depends on
      char avty[3 * (MQTT_MAX_TOPIC_LEN + 24)];
      int n = snprintf_P(avty, sizeof(avty), PSTR("{\"t\":\"%s/status\"}"), mqttDeviceTopic);
      if (e.needs & NEEDS_BMP) n += snprintf_P(avty + n, sizeof(avty) - n, PSTR(",{\"t\":\"%s/bmp280/status\"}"), mqttDeviceTopic);
      if (e.needs & NEEDS_DHT) snprintf_P(avty + n, sizeof(avty) - n, PSTR(",{\"t\":\"%s/dht11/status\"}"), mqttDeviceTopic);

      char devName[70];
      jsonEscape(devName, serverDescription, sizeof(devName));
      const char* m = escapedMac.c_str();         // "aabbccddeeff"
      snprintf_P(payload, sizeof(payload), PSTR(
        "{\"name\":\"%s\",\"uniq_id\":\"wled_%s_%s\",\"obj_id\":\"wled_%s_%s\","
        "\"stat_t\":\"%s\",\"unit_of_meas\":\"%s\",\"dev_cla\":\"%s\",\"stat_cla\":\"measurement\","
        "\"sug_dsp_prec\":1,\"avty_mode\":\"all\",\"avty\":[%s],"
        "\"dev\":{\"cns\":[[\"mac\",\"%.2s:%.2s:%.2s:%.2s:%.2s:%.2s\"]],\"name\":\"%s\",\"mf\":\"WLED\","
        "\"mdl\":\"ESP32-S3 SuperMini + BMP280 + DHT11\",\"sw\":\"%s\"}}"),
        e.name, m, e.key, m, e.key,
        state, e.unit, e.devClass,
        avty,
        m, m + 2, m + 4, m + 6, m + 8, m + 10, devName,
        versionString);
      mqtt->publish(topic, 0, true, payload);
    }

    void publishDiscovery() {
      if (!WLED_MQTT_CONNECTED || !mqttDeviceTopic[0] || escapedMac.length() < 12) return;
      for (const Entity& e : ENTITIES) publishDiscoveryEntity(e);
    }

    void publishStatus(const char* sensor, bool present) {
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      snprintf_P(topic, sizeof(topic), PSTR("%s/%s/status"), mqttDeviceTopic, sensor);
      mqtt->publish(topic, 0, true, present ? "online" : "offline");
    }

    void publishAvailability() {
      if (!mqttReady()) return;
      publishStatus("bmp280", bmpPresent);
      publishStatus("dht11",  dhtPresent);
      lastPublishedBmp = bmpPresent;
      lastPublishedDht = dhtPresent;
    }

    void publishAvailabilityIfChanged() {
      if (bmpPresent != lastPublishedBmp || dhtPresent != lastPublishedDht) publishAvailability();
    }

    void publishValue(const Entity& e, float v) {
      char topic[MQTT_MAX_TOPIC_LEN + 24];
      char value[16];
      stateTopic(topic, sizeof(topic), e);
      snprintf_P(value, sizeof(value), PSTR("%.1f"), v);
      mqtt->publish(topic, 0, true, value);
    }

    void publishDewPoint(uint32_t now) {
      if (bmpReadingFresh(now) && dhtReadingFresh(now)) publishValue(ENTITIES[E_DEW], dewPoint());
    }

    void publishBmpReadings() {
      const uint32_t now = millis();
      if (!mqttReady() || !bmpReadingFresh(now)) return;
      publishValue(ENTITIES[E_TEMP],  temperatureC);
      publishValue(ENTITIES[E_PRESS], pressureHPa);
      publishDewPoint(now);
    }

    void publishDhtReadings() {
      const uint32_t now = millis();
      if (!mqttReady() || !dhtReadingFresh(now)) return;
      publishValue(ENTITIES[E_HUM], humidityPct);
      publishDewPoint(now);
    }

    void publishReadings() {
      publishBmpReadings();
      publishDhtReadings();
    }
#else
    void publishAvailabilityIfChanged() {}
    void publishBmpReadings() {}
    void publishDhtReadings() {}
#endif
};

constexpr WledAssitant::Entity WledAssitant::ENTITIES[];
const char WledAssitant::_name[] PROGMEM = "wled_assitant";

static WledAssitant wled_assitant;
REGISTER_USERMOD(wled_assitant);
