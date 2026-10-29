---
description: usermods/ knowledge — wled_assitant usermod lifecycle, WLED/arduino-esp32 facts verified in source
applyTo: "usermods/**"
---

# AGENTS (usermods/wled_assitant/)

## OVERVIEW

Single-file usermod `wled_assitant.cpp` (class `WledAssitant`, registered with `REGISTER_USERMOD`). Owns: BMP280 state machine, optional MQTT/HA discovery, `/json/info` diagnostics, and the USB RX queue size. It never renders LEDs. LED data is driven directly from GPIO5 (3.3V logic, no level shifter) through a 220Ω resistor into DIN — a hardware/voltage choice (PSU trimmed to 4.75V), nothing the usermod controls.

## WLED v16.0.1 LIFECYCLE (verified in wled00/wled.cpp)

1. C++ static init → usermod **constructor** (earliest hook without core patches)
2. `Serial.begin(115200)` → FS mount → `deserializeConfigFromFS()` (calls `readFromConfig()` only if cfg has `"um"`; does `Wire.begin` on global I2C pins)
3. `beginStrip()` → RMT bus created on GPIO5 (`PinOwner::BusDigital`), black frame shown
4. `UsermodManager::setup()` → Wi-Fi → `serialCanRX/TX` computed → web server
5. `loop()`: `handleSerial()` (Adalight) → … → `UsermodManager::loop()` → `strip.service()` (skipped while realtime)

## USB / ADALIGHT FACTS

- `ARDUINO_USB_MODE=1` + `ARDUINO_USB_CDC_ON_BOOT=1` ⇒ `Serial` is `HWCDC` (USB-Serial/JTAG). Stock `handleSerial()` parses Adalight from it.
- HWCDC ISR **drops bytes when its RX queue is full** (no flow control); default queue is 256 B < one 438 B frame. The constructor calls `Serial.setRxBufferSize(WLEDA_USB_RX_BUFFER)` (2048) **before** `Serial.begin()`; `HWCDC::begin()` keeps an existing queue. Never resize later — it would swap the queue under the live ISR.
- Realtime fallback is stock: `realtimeLock()` per completed frame, `exitRealtime()` after `realtimeTimeoutMs`.

## BMP280 RULES

- BMP280 only (temperature + pressure, **no humidity**); one sensor driver by owner decision 2026-10-29. Direct Wire transactions, no library; `Wire.setTimeOut(15)`; one short transaction per `loop()` step; 3 consecutive failures ⇒ "missing", re-probe (reset + calibration) each interval.
- Probe: chip ID 0xD0 at 0x76 then 0x77; 0x56–0x58 accepted. Others are rejected with `last_fault` `id 60 @76, not BMP280`.
- Register reads use a repeated start (`endTransmission(false)`; supported in arduino-esp32 2.0.18 / Tasmota platform 2024.06.00).
- After probe: soft reset 0xE0←0xB6, wait for status 0xF3 bit0 (im_update) clear, read calibration 0x88..0x9F (24 B); reject `T1 == 0 || P1 == 0` as blank.
- Measure: forced mode, x1 oversampling (`ctrl_meas` 0xF4 = 0x25), wait 8 ms (max 6.4 ms), poll status bit3. Data 0xF7: press[3] temp[3]. Raw 0x80000 = reset value ⇒ reject.
- Compensation = Bosch integer reference code (int32 T, int64 P). Host-checked: DS §3.12 example → T 2508, t_fine 128422, P 25767233 (100653.25 Pa; the float formula gives the same, not ".27"). On hardware 2026-10-30: 26.5 °C, 990.5 hPa, 0 errors.
- Offsets are applied after compensation (`t_fine` stays uncorrected for P).
- Trap: `/json/cfg` POST of `hw.if.i2c-pin` sets pins to −1 once `Wire` is running (`Wire.setPins` fails in `deserializeConfig`); change them with POST `/settings/um` (`SDA`, `SCL`, `MOSI`, `MISO`, `SCLK`, `RBT`).
- Trap: acetone/IPA killed the previous HTU21D's humidity channel (−5 %RH with valid CRC, later `00 00 00`). Don't clean exposed sensors with solvents.

## MQTT / HA RULES

- Guard with `#ifndef WLED_DISABLE_MQTT` and `WLED_MQTT_CONNECTED`; never connect/reconnect ourselves.
- `onMqttConnect()` runs in the AsyncMqttClient task → only set `discoveryDirty`; publish from `loop()` (static buffers are not thread-safe).
- Discovery: `<prefix>/sensor/wled_<mac>/{temperature,pressure}/config`, retained; device `cns: [["mac", …]]` merges with the native WLED HA device; availability = WLED LWT `<topic>/status` + `<topic>/bmp280/status` (`avty_mode: all`). Empty retained payload removes entities.
- State topics `<topic>/bmp280/<leaf>` (were `<topic>/htu21d/…`; old retained topics are left behind). Temperature's unique ID is unchanged from the HTU21D, so HA keeps its history.

## CONVENTIONS

- Pins only via build flags (`WLEDA_LED_DATA_PIN`, `I2CSDAPIN`, `I2CSCLPIN`); `static_assert`s reject reserved/duplicate pins — keep them when adding pins.
- Config lives under cfg.json `um.wled_assitant` (`bmpEnabled`; the old `htuEnabled` key is dropped on the next save); `readFromConfig()` returns false when keys are missing so WLED re-saves defaults.
- No per-frame or per-publish heap allocation; comments only where lifecycle/concurrency is non-obvious.
- Usermod API reference: `wled/wled00/fcn_declare.h` (`class Usermod`). Inspect source before using any hook — don't trust old usermod examples.
