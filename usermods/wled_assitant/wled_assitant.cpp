/*
 * wled-assitant usermod — ESP32-S3 SuperMini desk ambient light
 *
 *  - TXS0108E OE lifecycle (translator held off until WLED's LED output is initialised and idle)
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
    }

    void loop() override {
      const uint32_t now = millis();
      serviceOe(now);
    }

    void onUpdateBegin(bool init) override {
      // Keep the strip isolated while flash is being written; re-enabled by serviceOe() on failure.
      oeSuspended = init;
      if (init) setOe(false);
    }

    void addToJsonInfo(JsonObject& root) override {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray oe = user.createNestedArray(F("LED level shifter"));
      oe.add(oeStatusText());

      JsonArray usb = user.createNestedArray(F("USB Adalight"));
      usb.add(usbStatusText());

      // machine-readable copy for scripts / HA REST sensors
      JsonObject d = root.createNestedObject(F("wled_assitant"));
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
      top[F("oeDelayMs")] = oeDelayMs;
    }

    bool readFromConfig(JsonObject& root) override {
      JsonObject top = root[FPSTR(_name)];
      bool complete = !top.isNull();
      complete &= getJsonValue(top[F("oeDelayMs")], oeDelayMs, uint16_t(0));

      if (oeDelayMs > 5000) oeDelayMs = 5000;
      return complete;
    }

    void appendConfigData(Print& s) override {
      s.print(F("addInfo('wled_assitant:oeDelayMs',1,'ms extra hold before OE goes high (GPIO"));
      s.print(WLEDA_TXS_OE_PIN);
      s.print(F(", fixed at build time)');"));
    }

    uint16_t getId() override { return USERMOD_ID_UNSPECIFIED; }

  private:
    static const char _name[];

    // ---- settings (cfg.json "um" → "wled_assitant") ----
    uint16_t oeDelayMs = 0;

    // ---- OE state ----
    bool     oeAllocated = false;
    bool     oeEnabled   = false;
    bool     oeSuspended = false;
    uint32_t setupMs     = 0;
    uint32_t oeEnabledAt = 0;

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
};

const char WledAssitant::_name[] PROGMEM = "wled_assitant";

static WledAssitant wled_assitant;
REGISTER_USERMOD(wled_assitant);
