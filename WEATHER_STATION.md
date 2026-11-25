# Weather Station (native Bruce app)

The Weather Station app decodes 433/315/868/915 MHz OOK weather sensors and
shows a live station list. It also decodes saved Flipper/Bruce `.sub` captures
offline, so sample files can be verified without a transmitter.

## Supported protocols

| Protocol | Frame | Notes |
| --- | --- | --- |
| **AcuRite 606TX** | 32-bit PPM | LFSR checksum (0x98/0xF1); temperature, channel, battery |
| **AcuRite TXR family** | 56/64/72/80-bit PWM | Tower/592TXR (56), 3n1 (64), 5n1 (64), 6045M (72), Atlas (80); checksum + even parity |
| **Nexus / Nexus-TH** | 36-bit PPM | repeated-frame decode; 0xFE/0xFF/0xCC humidity sentinels accepted |
| **Prologue** | 36-bit PPM | repeated-frame decode |
| **LaCrosse TX141TH-Bv2/Bv3** | 40-bit PWM | reflected LFSR checksum (0x31/0xF4); temp/humidity/channel |

The AcuRite TXR family covers the **02077M** captures (56-bit Tower frames),
the **3n1/5n1** stations and the **6045M** lightning sensor. Atlas messages
with temperature/humidity/wind are also decoded.

## Live tuning

Defaults after a clean install:

- **433.920 MHz**
- **406 kHz RX bandwidth** — wide enough to hear both common weather
  sub-bands at once: **433.79 MHz** (Nexus/LaCrosse) and **434.10 MHz**
  (AcuRite 606TX).
- **Default AGC** — nearby sensors are strong; Max Sensitivity can overload
  and distort them. Max Sensitivity remains available for weak/distant
  sensors.

Settings → Frequency also has direct presets for 433.790, 434.100, 433.889,
433.420, 315, 868.35 and 915 MHz, plus a custom kHz entry. Settings are saved
to `/BruceRF/weather.conf`.

If a sensor is very weak, increase the bandwidth and/or switch AGC to
Max Sensitivity.

## Live decoding

The live screen keeps a 512-pulse carry-over between RMT captures, so a frame
split at a capture boundary still has its full bit train. Checksum-validated
protocols (AcuRite/Lacrosee) decode from a single frame; Nexus/Prologue still
require 3/4 identical repeats to avoid false positives.

## Decode a saved `.sub` file

1. Copy the Flipper folders (for example `Weather_stations`) to the SD card
   root.
2. **RF → Weather Station → Decode .sub file**.
3. Browse to the file and select it.

The app accepts RAW captures (`RAW_Data:`), including multi-line files. It
shows model, ID, channel, temperature, humidity, wind/rain and battery. If no
protocol matches, it shows an extraction summary (`TXR/LaC/606/Nex/Por`
counts) to help diagnose the capture.

BinRAW key files (`Protocol: BinRAW` / `Data_RAW:`) are not pulse captures and
are rejected with an explanatory message. Files larger than 512 KB are
rejected to avoid loading long recordings into RAM.

## Sample validation

The host regression test in `tests/weather_codec` runs the real decoder module
against these captures (default path `C:\Users\admin\Desktop\Weather_stations`):

| Sample | Result |
| --- | --- |
| `Nexus-TH/nexus-th_raw.sub` | Nexus-TH, ID 71, CH 1, 29.5 °C, 40 % |
| `VITEK_VT-3531/T0.sub` | Nexus-T, ID 71, CH 1, 2.4 °C (humidity sentinel) |
| `VITEK_VT-3531/T1.sub` | Nexus-TH, ID 71, CH 1, 2.6 °C, 39 % |
| `LaCrosse-TX141TH-BV2/...-raw.sub` | LaCrosse-TX141TH, ID 18, CH 1, 16.1 °C, 50 % |
| `Acurite_02077M/...18.04.09.sub` | AcuRite-Tower, ID 6315, CH 3, 5.2 °C, 83 % |
| `Acurite_02077M/...18.01.44.sub` | AcuRite-Tower, ID 6315, CH 3, 1.9 °C, 79 % |
| synthetic 606TX | AcuRite-606TX, ID 0x84, 13.1 °C |

Run it with:

```powershell
cd C:\Users\admin\Bruce-RF-Custom-Build\firmware
g++ -std=c++17 -O1 -I tests\weather_codec -I src\modules\rf `
  tests\weather_codec\test.cpp src\modules\rf\rf_weather_decoders.cpp `
  -o tests\weather_codec\weather_codec_test.exe
.\tests\weather_codec\weather_codec_test.exe
```

or `powershell -File tests\weather_codec\run.ps1`.

## Limitations

- **UNNI-UN0581B** samples are BinRAW key files, not RAW pulse captures, so
  they cannot be decoded without a RAW re-capture or the protocol definition.
- FSK/Manchester weather variants (some AcuRite 5n1/Atlas revisions) are not
  supported; only OOK PPM/PWM.
- The 02077M sample is a **Tower** (temperature/humidity); wind/rain are shown
  only for 3n1/5n1/Atlas frames.
- Nexus/Prologue have no checksum, so live decoding deliberately requires
  repeated identical frames. File decoding accepts a single frame.
- The app does not transmit.

## Build / flash

```powershell
pio run -e lilygo-t-embed-cc1101
pio run -e lilygo-t-embed-cc1101 -t upload --upload-port COMx
```

Or flash the factory image:

```powershell
esptool --chip esp32s3 --port COMx --baud 921600 write-flash 0x0 Bruce-lilygo-t-embed-cc1101.bin
```
