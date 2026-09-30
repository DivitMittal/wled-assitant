---
description: usermods/ knowledge — wled_assitant usermod lifecycle, WLED/arduino-esp32 facts verified in source
applyTo: "usermods/**"
---

# AGENTS (usermods/wled_assitant/)

## OVERVIEW

Single-file usermod `wled_assitant.cpp` (class `WledAssitant`, registered with `REGISTER_USERMOD`). Owns: TXS0108E OE lifecycle, HTU21D state machine, optional MQTT/HA discovery, `/json/info` diagnostics, and the USB RX queue size. It never renders LEDs.

## WLED v16.0.1 LIFECYCLE (verified in wled00/wled.cpp)

1. C++ static init → usermod **constructor** (earliest hook without core patches)
2. `Serial.begin(115200)` → FS mount → `deserializeConfigFromFS()` (calls `readFromConfig()` only if cfg has `"um"`; does `Wire.begin` on global I2C pins)
3. `beginStrip()` → RMT bus created on GPIO6 (`PinOwner::BusDigital`), black frame shown
4. `UsermodManager::setup()` → Wi-Fi → `serialCanRX/TX` computed → web server
5. `loop()`: `handleSerial()` (Adalight) → … → `UsermodManager::loop()` → `strip.service()` (skipped while realtime)

## USB / ADALIGHT FACTS

- `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1` ⇒ `Serial` is `HWCDC` (USB-Serial/JTAG). Stock `handleSerial()` parses Adalight from it.
- HWCDC ISR **drops bytes when its RX queue is full** (no flow control); default queue is 256 B < one 438 B frame. The constructor calls `Serial.setRxBufferSize(WLEDA_USB_RX_BUFFER)` (2048) **before** `Serial.begin()`; `HWCDC::begin()` keeps an existing queue. Never resize later — it would swap the queue under the live ISR.
- Realtime fallback is stock: `realtimeLock()` per completed frame, `exitRealtime()` after `realtimeTimeoutMs`.

## OE RULES

- Constructor: latch level 0, then `gpio_config` output + internal pull-down; `esp_register_shutdown_handler` drives LOW on `esp_restart()`.
- `setup()`: `PinManager::allocatePin(OE, true, PinOwner::UM_Unspecified)`; on failure OE stays LOW forever (reported as pin conflict).
- Enable only when GPIO6 owner is `BusDigital` **and** `!strip.isUpdating()` **and** `oeDelayMs` elapsed; then `strip.trigger()`. Drop LOW if the bus loses GPIO6, and during OTA (`onUpdateBegin`).
- No fixed boot delays; sensor/MQTT state must never gate OE.

## HTU21D RULES

- Direct Wire transactions, no library: soft reset 0xFE (15 ms), no-hold T 0xF3 (≤50 ms), RH 0xF5 (≤16 ms); NACK = still converting → re-poll after 10 ms.
- CRC-8 poly 0x31 init 0 (datasheet vectors 0x683A→0x7C, 0x4E85→0x6B). **Do not** validate the LSB status bit — datasheet samples contradict the documented meaning.
- `Wire.setTimeOut(15)`; one short transaction per `loop()` step; 3 consecutive failures ⇒ "missing", re-probe each interval.

## MQTT / HA RULES

- Guard with `#ifndef WLED_DISABLE_MQTT` and `WLED_MQTT_CONNECTED`; never connect/reconnect ourselves.
- `onMqttConnect()` runs in the AsyncMqttClient task → only set `discoveryDirty`; publish from `loop()` (static buffers are not thread-safe).
- Discovery: `<prefix>/sensor/wled_<mac>/{temperature,humidity}/config`, retained; device `cns: [["mac", …]]` merges with the native WLED HA device; availability = WLED LWT `<topic>/status` + `<topic>/htu21d/status` (`avty_mode: all`). Empty retained payload removes entities.

## CONVENTIONS

- Pins only via build flags (`WLEDA_LED_DATA_PIN`, `WLEDA_TXS_OE_PIN`, `I2CSDAPIN`, `I2CSCLPIN`); `static_assert`s reject reserved/duplicate pins — keep them when adding pins.
- Config lives under cfg.json `um.wled_assitant`; `readFromConfig()` returns false when keys are missing so WLED re-saves defaults.
- No per-frame or per-publish heap allocation; comments only where lifecycle/concurrency is non-obvious.
- Usermod API reference: `wled/wled00/fcn_declare.h` (`class Usermod`). Inspect source before using any hook — don't trust old usermod examples.
