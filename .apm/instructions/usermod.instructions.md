---
description: usermods/ knowledge — wled_assitant usermod lifecycle, WLED/arduino-esp32 facts verified in source
applyTo: "usermods/**"
---

# AGENTS (usermods/wled_assitant/)

## OVERVIEW

Single-file usermod `wled_assitant.cpp` (class `WledAssitant`, registered with `REGISTER_USERMOD`). Owns: BMP280 and DHT11 state machines, dew point, optional MQTT/HA discovery, `/json/info` diagnostics, and the USB RX queue size. It never renders LEDs. LED data is driven directly from GPIO5 (3.3V logic, no level shifter) through a 220Ω resistor into DIN — a hardware/voltage choice (PSU trimmed to 4.75V), nothing the usermod controls.

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

## DHT11 RULES

- Owner decision 2026-11-03: BMP280 = temperature + pressure, DHT11 = humidity only. The DHT11's own temperature (±2 °C, 1 °C steps on most units) is diagnostics (`dht11.temp_c`), never the Temperature entity, and is not used to correct RH (its error would exceed the correction). Dew point = Magnus (Sonntag: 17.62 / 243.12) from BMP280 T + DHT11 RH, only when both are fresh.
- **Never bit-bang.** A DHT11 frame is ~4 ms; masking interrupts that long starves WLED's RMT LED refill ISR. Read path: 20 ms start low timed across `loop()` calls (GPIO open-drain + pull-up), then RMT RX channel 4 (LL index 0, memory block 4, 1 µs ticks, 2 µs glitch filter, 1 ms idle ends the capture) records the reply in hardware.
- **No IDF RMT driver, no RMT interrupt.** `lib/NeoESP32RmtHI` (WLED's LED driver) takes `ETS_RMT_INTR_SOURCE` exclusively at level 5 (`esp_intr_alloc`, not shared) and zeroes `RMT.int_ena` on first install; `rmt_driver_install` would need its own ISR, and legacy `rmt_rx_start` dereferences driver state. So the channel is set up with `hal/rmt_ll.h` only and completion is polled from `RMT.int_raw` (rx_end bit 16+n, rx_err bit 20+n); our `int_ena` bits stay 0. Only write `int_clr` with our own bits.
- The RMT group clock is shared with the LED TX channels: read `sys_conf` (sclk_sel 1 = APB 80 MHz, 3 = XTAL; ÷(1+div_num)) to derive ticks; only configure it when `clk_en` is 0 (RMT unused). Rewriting it would glitch a running LED frame. S3 RX carrier demodulation defaults **on** — disable it.
- Decode: the data bits are the last 40 high pulses that end in a low (skips the release high and the 80 µs response, ignores a trailing idle high); "1" if high ≥ 48 µs. Reject checksum errors, all-zero frames (pass the checksum) and tenths digits > 9 (a DHT22 sends 16-bit values: `not a DHT11`). Host-tested 2026-11-03 with synthetic captures (with/without release high, idle high, 60/22 µs clone timing, absent sensor, short frame, bad pulse, checksum, DHT22, negative T).
- Poll pacing follows the BMP280 (`interval`, first read 2.5 s after boot — DHT11 needs 1 s after power-up); 3 consecutive failures ⇒ missing, still retried every interval (hot-plug works). `last_fault` `no response` = capture finished with no reply (wiring/power); `capture timeout` = RMT never reported an end (RMT problem).

## MQTT / HA RULES

- Guard with `#ifndef WLED_DISABLE_MQTT` and `WLED_MQTT_CONNECTED`; never connect/reconnect ourselves.
- `onMqttConnect()` runs in the AsyncMqttClient task → only set `discoveryDirty`; publish from `loop()` (static buffers are not thread-safe).
- Discovery: `<prefix>/sensor/wled_<mac>/{temperature,pressure,humidity,dew_point}/config`, retained (table `ENTITIES`); device `cns: [["mac", …]]` merges with the native WLED HA device; availability = WLED LWT `<topic>/status` + the status topic of each sensor the value needs (`<topic>/bmp280/status`, `<topic>/dht11/status`; `avty_mode: all`). Empty retained payload removes entities — also sent when the entity's sensor is disabled.
- State topics `<topic>/bmp280/{temperature,pressure}`, `<topic>/dht11/humidity`, `<topic>/dew_point` (were `<topic>/htu21d/…`; old retained topics are left behind). Temperature and humidity unique IDs are unchanged from the HTU21D, so HA keeps their history.

## CONVENTIONS

- Pins only via build flags (`WLEDA_LED_DATA_PIN`, `I2CSDAPIN`, `I2CSCLPIN`, `WLEDA_DHT_PIN`); `static_assert`s reject reserved/duplicate pins — keep them when adding pins.
- Config lives under cfg.json `um.wled_assitant` (`bmpEnabled`, `dhtEnabled`, `humOffset`; the old `htuEnabled` key is dropped on the next save); `readFromConfig()` returns false when keys are missing so WLED re-saves defaults.
- No per-frame or per-publish heap allocation; comments only where lifecycle/concurrency is non-obvious.
- Usermod API reference: `wled/wled00/fcn_declare.h` (`class Usermod`). Inspect source before using any hook — don't trust old usermod examples.
