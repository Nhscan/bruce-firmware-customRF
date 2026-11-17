# RF Tools for Bruce (native port)

This fork carries a native Bruce implementation of the Flipper Zero **RF Tools**
app, built for the LilyGO T-Embed CC1101 (and other Bruce boards with a CC1101).
It lives under the existing **RF → RF Tools** menu and does not require the
BruceJS interpreter, an SD script, or a separate app package.

The goal is feature parity with the Flipper app where the hardware allows it:

| Flipper RF Tools | Bruce RF Tools | Notes |
| --- | --- | --- |
| Capture (Read / Read RAW) | **Capture (Read)** / **Capture RAW** | Reuses Bruce's `RFScan` engine. |
| Repeater | **Repeater** | Reuses `rf_repeater`. |
| Bruteforce | **Bruteforce** | Reuses `rf_bruteforce`. |
| Frequency Analyzer | **Frequency Analyzer** | Reuses the SubGHZ analyzer. |
| Rolling Codes | **Rolling Codes** | Capture a set, replay it in order. |
| Key Info | **Key Info** | Capture and explain a signal. |
| Car Keys | **Car Keys** | Honda V1, Chrysler V0, Kia V1, Fiat V2. |
| Signal Meter | **Signal Meter** | Reuses `rf_CC1101_rssi`. |
| Saved Signals | **Saved Signals** | Reuses `sendCustomRF`. |
| Radio Settings | **Radio Settings** | Frequency, RX BW, AGC, repeat, counter increment, auto-save. |
| Add Manually | **Add Manually** | Reuses the SubGHZ generator (fixed + KeeLoq). |

## Car Keys

Car Keys is the part that does not exist in stock Bruce. It ports the four
ProtoPirate automotive protocols used by RF Tools:

| Protocol | Frame | Fields | Encoder |
| --- | --- | --- | --- |
| Honda V1 | 68-bit PWM, 1 ms / 2 ms | serial (28-bit), button, 16-bit counter, checksum | yes |
| Chrysler V0 | 80-bit packet + 2-byte tail | encoded block A/B, 6-bit counter, button, seed | yes |
| Kia V1 | 90-pulse header + 57-bit Manchester | serial (32-bit), button, 12-bit counter, 4-bit CRC | yes (rewritten, see below) |
| Fiat V2 | 112-bit Manchester, 14 raw bytes | UID, button, counter, Hop, raw bytes | no — RAW replay |

Behaviour, matching the Flipper app:

- **Listen / Capture** runs continuously with a rolling 4096-edge RAM buffer.
  Noise overwrites the oldest edges, so the app never restarts and a fob press
  is caught on the first try. The screen says `Listening... press anytime`.
- A decode is shown with protocol, serial, button, counter (and Hop for Fiat),
  then auto-saved (if enabled) as:
  - `/BruceRF/CarKeys/ck_NNN_<Protocol>_Btn<xx>_Cnt<xxxx>.sub` — RAW capture
  - `…​.key` — Flipper-compatible decoded snapshot for Honda/Chrysler/Kia
- Every decode is appended to `/BruceRF/CarKeys/car_log.txt` with a counter
  delta against the previous press of the same protocol.
- **Send decoded** re-encodes the frame and advances the counter by the
  `Counter incr` setting. **Send RAW** replays the captured edges. Fiat only
  offers the RAW path because its Hop field is keyed and cannot be generated
  from captures.
- **Archive to Keys/** copies the capture and its `.key` companion into
  `/BruceRF/Keys/`.
- **Replay list** lists every `ck_*.sub`. It prefers the `.key` companion and
  falls back to the RAW capture. The frequency stored in the `.sub` is used for
  the transmission, not the current menu frequency.
- **Delete replay list** removes both the `ck_*.sub` and matching `ck_*.key`.

## Protocol notes / differences from the Flipper source

- **Kia V1**: the upstream ProtoPirate encoder and decoder do not agree with
  each other (the encoder emits a bare start half-pulse and the decoder's
  Manchester event polarity is inverted), and real Kia fobs were never verified
  upstream. This port uses a clean, self-consistent Manchester format:
  90 × (low 1600 µs, high 1600 µs), a low separator, then 57 Manchester bits
  (start bit 1 + the 56-bit payload, MSB first). The decoder collects cells
  directly, which avoids the one-bit phase lag of the generic Flipper
  Manchester state machine.
- **Fiat V2** is decode + RAW replay only. Generating the next valid rolling
  frame requires the manufacturer's keyed Hop algorithm; captures are not
  enough. The raw capture is the reliable path.
- **Chrysler V0** keeps both plain blocks (counter block and serial block) so
  the encoder can rebuild the pair exactly. If a file has only one block, the
  encoder falls back to the ProtoPirate behaviour (mirror the one block).
- The automotive decoders are deliberately **not** added to Bruce's generic OOK
  registry. They run only inside the Car Keys/Key Info/Rolling screens, so
  ordinary 433 MHz decoding stays fast and does not produce false automotive
  matches.

## Files

```
src/modules/rf/rft/
  rft.h              shared frame/type API + settings
  rft_decode.cpp     Honda / Chrysler / Kia / Fiat decoders + summary
  rft_encode.cpp     Honda / Chrysler / Kia encoders (Fiat has none)
  rft_keyfile.cpp    .key text parse/serialize
  rft_car.cpp        Car Keys UI, listening, save/log, replay list, settings
  rft_tools.cpp      RF Tools menu, Key Info, Rolling Codes
tests/rft_codec/     host-side round-trip / synthetic-frame regression test
```

Integration points:

- `src/core/menu_items/RFMenu.cpp` — new **RF Tools** entry.
- `src/modules/rf/rf_subghz.{h,cpp}` — exported `rf_frequency_analyzer()` and
  `rf_add_manually()` so the new menu reuses the tested screens.

## Build and flash

```powershell
cd C:\Users\admin\Bruce-RF-Custom-Build\firmware
pio run -e lilygo-t-embed-cc1101
```

Outputs:

- `Bruce-lilygo-t-embed-cc1101.bin` — full factory image (flash at `0x0`)
- `.pio\build\lilygo-t-embed-cc1101\firmware.bin` — app-only image (flash at `0x10000`)

Upload directly:

```powershell
pio run -e lilygo-t-embed-cc1101 -t upload --upload-port COMx
```

## Regression test

```powershell
powershell -ExecutionPolicy Bypass -File tests\rft_codec\run.ps1
```

The test builds the real `rft_decode.cpp` / `rft_encode.cpp` /
`rft_keyfile.cpp` with a tiny Arduino `String` shim and checks:

- Honda / Kia encode → decode round trips
- a synthetic 80-bit Chrysler packet
- synthetic Fiat frames with separate and merged (air-like) pulses
- Honda decode with ±100 µs timing jitter
- `.key` serialize → parse round trip

Current result: **ALL PASS**.

## Known limitations / on-device checklist

Real hardware was not available while this port was built, so the following
must be checked on the T-Embed:

1. `RF → RF Tools` opens and every entry runs/returns cleanly.
2. Car Keys listens through ambient noise for several minutes; heap stays
   stable and the screen keeps updating.
3. Fiat: press lock/unlock at 433.92 MHz / AM650. Confirm the decode shows
   `Fiat V2`, UID, `Btn`, `Cnt`, `Hop`, that `ck_*.sub` is written, and that
   RAW replay transmits.
4. Honda/Chrysler/Kia (if a fob is available): confirm the result offers
   `Send decoded`, a `.key` companion appears, and repeated sends advance `Cnt`
   by the configured increment.
5. Replay list: confirm each entry sends at its own captured frequency and
   returns to the list.
6. Delete replay list removes both `.sub` and `.key`.

Fiat rolling-code generation, real-fob timing tolerance, and whether the
rewritten Kia wire format matches a real Kia receiver are the open questions.
