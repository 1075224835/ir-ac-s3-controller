# ESP32-S3 IR AC Controller

Firmware skeleton for a GOOUUU ESP32-S3 board with:

- IR transmitter module
- IR receiver module
- SHT31 temperature/humidity sensor
- Web configuration page
- Built-in AC protocol control through `IRremoteESP8266`
- Raw IR learning and replay for unsupported remotes
- Sleep temperature curve automation

## Default Wiring

Change pins in [include/AppConfig.h](include/AppConfig.h) if your board wiring differs.

| Module | ESP32-S3 GPIO |
| --- | --- |
| IR TX module IN | GPIO 4 |
| IR RX module OUT | GPIO 14 |
| SHT31 SDA | GPIO 8 |
| SHT31 SCL | GPIO 9 |
| 3V3 / GND | 3V3 / GND |

Use a driven 940 nm IR LED transmitter module for useful range. A GPIO pin driving an IR LED directly is usually too weak. Most common IR receiver modules are 38 kHz demodulated receivers.

## Build And Flash

Install PlatformIO, then run:

```powershell
pio run
pio run -t upload
pio device monitor -b 115200
```

If this GOOUUU board does not map well to `esp32-s3-devkitc-1`, change the `board` value in [platformio.ini](platformio.ini).

## First Run

1. Flash the firmware.
2. The firmware first tries the default home WiFi defined in [include/AppConfig.h](include/AppConfig.h).
3. If STA connection fails for about 15 seconds, connect to AP `IR-AC-S3`, password `12345678`, then open `http://192.168.4.1/`.
4. If STA connects, open the configured static IP `http://192.168.0.57/` or the IP shown in serial/status.
5. Select an AC protocol and model in `AC Library Control`, then test power/cool/temperature.
6. If the protocol is unknown, press buttons on the physical remote and save captures under `Learn Remote / Build Raw Library`.

## WiFi Behavior

The WiFi setup follows the companion `64x64-esp32-s3` project:

- default STA SSID/password and static IP settings can be set by copying [include/AppSecrets.example.h](include/AppSecrets.example.h) to `include/AppSecrets.local.h`; the local file is ignored by git
- hostname is set before connecting
- WiFi power saving is disabled for more stable web/IR timing
- the device starts in STA mode when an SSID is configured
- AP fallback starts after a connection timeout
- once STA connects, the AP is stopped
- the web page can save a new WiFi config or forget WiFi and return to AP-only mode

## Built-In Protocol Mode

When `IRremoteESP8266` supports the AC protocol, the firmware sends complete AC state messages:

- power
- mode
- setpoint
- fan speed
- quiet/light/beep/econo defaults

This is the preferred path because the sleep curve can generate arbitrary setpoints.

## Raw Library Mode

If the built-in library cannot control the AC, save raw commands with metadata such as:

- `cool 27 auto`
- `cool 26 auto`
- `cool 25 auto`
- `off`

The automation will pick the closest learned raw command for the target setpoint. Unknown protocol hash values are not used for replay; raw timings are stored and replayed with `sendRaw()`.

## Sleep Curve

Default curve:

```json
[
  {"minute":0,"temp":27},
  {"minute":90,"temp":26.5},
  {"minute":300,"temp":25.5},
  {"minute":480,"temp":26.5}
]
```

`minute` is minutes after the configured sleep start time. The firmware linearly interpolates between points. This is only a comfort starting point, not a medical model. Tune it for room insulation, bedding, AC capacity, and personal preference.

For real "night/morning" behavior, keep WiFi connected so the ESP32-S3 can sync time via NTP. Without NTP, the firmware falls back to minutes since boot.

## Limits

- AC remotes usually send a complete state every time, not a simple "temperature +1" button code.
- Demodulated IR receivers cannot tell the original carrier frequency. Raw replay defaults to 38 kHz; try 36/40/56/57 kHz if replay fails.
- The web server is intentionally the built-in synchronous `WebServer`; IR sends are queued and executed from `loop()` to avoid timing issues during long AC messages.
