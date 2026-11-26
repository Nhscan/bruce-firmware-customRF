// Native live weather-sensor monitor.
//
// Decodes AcuRite-606TX, the AcuRite TXR PWM family (Tower/3n1/5n1/6045M/
// Atlas), Nexus/Nexus-TH, Prologue and LaCrosse TX141TH-Bv2.
//
// Two modes:
//   - Live listen: continuous RX with a small carry-over buffer so a frame
//     split across RMT captures still decodes.
//   - Decode .sub file: browse the SD card and decode a Flipper/Bruce RAW
//     capture without transmitting anything.
#include "rf_weather_station.h"
#include "rf_weather_decoders.h"

#include "protocols/rf_decoder.h"
#include "rf_utils.h"

#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <globals.h>

namespace {

// Settings. RAM-backed by default, persisted to /BruceRF/weather.conf when
// changed so the next launch keeps the tuned frequency.
float g_freq = 433.92f;
// 406 kHz covers both common weather sub-bands around 433.92:
// 433.79 MHz (Nexus/LaCrosse) and 434.10 MHz (AcuRite 606TX). Narrower
// filters miss one of them unless the user retunes for each sensor.
float g_bw = 406.0f;
// Nearby weather sensors are strong; Max Sensitivity overloads/distorts them.
// It is still available in Settings for weak/distant sensors.
int g_agc = 0;
// 0 = Flipper OOK270 register set, 1 = Flipper OOK650 register set,
// 2 = Manual (use g_bw/g_agc). The Flipper `.sub` preset matters: the short
// AcuRite file is Ook270 and the long recordings are Ook650.
int g_preset = 1;
bool g_saveCaptures = false;
bool g_debug = false;
int g_saveIndex = 0;

constexpr const char *WEATHER_CONF = "/BruceRF/weather.conf";

const char *agc_label(int agc) {
    switch (agc) {
    case 1: return "AGC C7";
    case 2: return "AGC C3 (test)";
    case 3: return "Max Sensitivity";
    default: return "Default";
    }
}

void apply_agc(int agc) {
    if (agc == 1) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    else if (agc == 2) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC3);
    else if (agc == 3) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x07);
}

const char *preset_label(int preset) {
    switch (preset) {
    case 0: return "Flipper OOK270";
    case 1: return "Flipper OOK650";
    default: return "Manual BW/AGC";
    }
}

// Apply the matching Flipper CC1101 preset registers. The only differences
// that matter for these RAW files are the RX filter (270 vs 650 kHz) and the
// AGCCTRL0/2 gain profile.
void apply_preset() {
    if (g_preset == 2) {
        ELECHOUSE_cc1101.setRxBW(g_bw);
        apply_agc(g_agc);
        return;
    }
    const bool ook650 = (g_preset == 1);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG3, 0x32); // 3.79372 kBaud
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG4, ook650 ? 0x17 : 0x67);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FIFOTHR, 0x07);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, ook650 ? 0x91 : 0x40);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x00);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, ook650 ? 0x07 : 0x03);
    ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1, 0xB6);
}

void settings_load() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    File f = fs->open(WEATHER_CONF, FILE_READ);
    if (!f) return;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        const int eq = line.indexOf('=');
        if (eq < 0) continue;
        String k = line.substring(0, eq);
        String v = line.substring(eq + 1);
        k.trim();
        v.trim();
        if (k == "freq") g_freq = v.toFloat();
        else if (k == "bw") g_bw = v.toFloat();
        else if (k == "agc") g_agc = v.toInt();
        else if (k == "preset") g_preset = v.toInt();
        else if (k == "save") g_saveCaptures = v.toInt() != 0;
        else if (k == "debug") g_debug = v.toInt() != 0;
    }
    f.close();
}

void settings_save() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    if (!fs->exists("/BruceRF")) fs->mkdir("/BruceRF");
    File f = fs->open(WEATHER_CONF, FILE_WRITE, true);
    if (!f) return;
    String out;
    out += "freq=" + String(g_freq, 3) + "\n";
    out += "bw=" + String((int)g_bw) + "\n";
    out += "agc=" + String(g_agc) + "\n";
    out += "preset=" + String(g_preset) + "\n";
    out += "save=" + String(g_saveCaptures ? 1 : 0) + "\n";
    out += "debug=" + String(g_debug ? 1 : 0) + "\n";
    f.write((const uint8_t *)out.c_str(), out.length());
    f.close();
}

String read_text(FS *fs, const String &path) {
    File f = fs->open(path, FILE_READ);
    if (!f) return "";
    String s;
    while (f.available()) s += (char)f.read();
    f.close();
    return s;
}

String base_name(const String &path) {
    const int slash = path.lastIndexOf('/');
    return slash >= 0 ? path.substring(slash + 1) : path;
}

void save_capture(const std::vector<int> &durations, float freq, const String &label) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    if (!fs->exists("/BruceRF")) {
        if (!fs->mkdir("/BruceRF")) return;
    }
    char filename[64];
    snprintf(filename, sizeof(filename), "/BruceRF/weather_%d.sub", g_saveIndex++);
    File file = fs->open(filename, FILE_WRITE, true);
    if (!file) return;

    String rawData;
    bool hasCrc = false;
    uint64_t crc = 0;
    std::vector<int> indexed;
    int rawBits = 0, rawTe = 0;
    rf_build_raw(durations, rawData, hasCrc, crc, indexed, rawBits, rawTe);

    String out = rf_subghz_header(freq);
    out += "Preset: Ook270Async\n";
    out += "Protocol: RAW\n";
    out += "RAW_Data: " + rawData;
    file.write((const uint8_t *)out.c_str(), out.length());
    file.close();
    Serial.printf("[Weather] saved %s (%s)\n", filename, label.c_str());
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void draw_weather_screen(
    const std::vector<WeatherReading> &stations, const std::vector<int> &counts, int rssi, int reads,
    bool listening, bool fullRedraw) {
    if (fullRedraw) {
        tft.fillScreen(bruceConfig.bgColor);
        drawMainBorder();
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.setCursor(10, 26);
        tft.print("Weather Station");
        tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
        tft.setCursor(10, 40);
        tft.print(g_freq, 3);
        tft.print(" MHz");
    }

    int barMax = 120;
    int barW = max(0, min(barMax, (int)((rssi + 100) * (barMax / 70.0f))));
    tft.setTextSize(FP);
    tft.fillRect(10, 50, barMax + 2, 10, getColorVariation(bruceConfig.priColor));
    tft.fillRect(11, 51, barW, 8, bruceConfig.priColor);
    tft.fillRect(10 + barMax + 10, 50, tftWidth - (10 + barMax + 10) - 10, 10, bruceConfig.bgColor);
    tft.setCursor(10 + barMax + 10, 50);
    tft.print(rssi);
    tft.print(" dBm");

    // Weather readings use a bright second colour and a larger font so they
    // stand out from the system-coloured chrome/labels.
    const uint16_t accent = TFT_ORANGE;

    int y = 68;
    tft.fillRect(10, y, tftWidth - 20, tftHeight - y - 24, bruceConfig.bgColor);
    if (stations.empty()) {
        tft.setTextSize(FP);
        tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
        tft.setCursor(10, y);
        tft.print(listening ? "Listening... press anytime" : "No stations yet");
    } else {
        for (size_t i = 0; i < stations.size(); i++) {
            const auto &s = stations[i];
            const int count = (i < counts.size()) ? counts[i] : 1;
            const bool hasExtra = s.hasWind || s.hasRain;
            const int blockH = hasExtra ? 42 : 31;
            if (y + blockH > tftHeight - 24) break;

            // Label line: system colour, normal size.
            tft.setTextSize(FP);
            tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
            tft.setCursor(10, y);
            tft.print(s.model);
            tft.print(" #");
            tft.print(s.id);
            tft.print(" CH");
            tft.print(s.channel);
            tft.print(" x");
            tft.print(count);
            tft.print(s.battery ? " BATT" : " LOW");
            y += 11;

            // Main reading: bright orange, one size larger.
            tft.setTextSize(FP + 1);
            tft.setTextColor(accent, bruceConfig.bgColor);
            tft.setCursor(16, y);
            if (s.hasTemp) {
                tft.print(s.tempC, 1);
                tft.print("C ");
                tft.print(s.tempC * 9.0f / 5.0f + 32.0f, 1);
                tft.print("F ");
            } else {
                tft.print("-- ");
            }
            if (s.hasHumidity) {
                tft.print(s.humidity);
                tft.print("%RH");
            }
            y += 18;

            if (hasExtra) {
                tft.setTextSize(FP);
                tft.setTextColor(accent, bruceConfig.bgColor);
                tft.setCursor(16, y);
                if (s.hasWind) {
                    tft.print("W:");
                    tft.print(s.windKph, 0);
                    tft.print("km/h ");
                }
                if (s.hasRain) {
                    tft.print("R:");
                    tft.print(s.rainMm, 1);
                    tft.print("mm");
                }
                y += 11;
            }
            y += 2;
        }
    }
    tft.setTextSize(FP);
    tft.fillRect(10, tftHeight - 24, tftWidth - 20, 16, bruceConfig.bgColor);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setCursor(10, tftHeight - 16);
    tft.print("SEL menu  ESC exit  n=");
    tft.print(reads);
}

void show_text_screen(const String &title, const String &body) {
    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder();
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.setCursor(10, 26);
    tft.print(title);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setTextWrap(true);
    tft.setCursor(10, 44);
    tft.print(body);
    tft.setTextWrap(false);
    tft.setCursor(10, tftHeight - 18);
    tft.print("Press any key");
    delay(200);
    bool esc = false;
    while (true) {
        if (check(EscPress)) {
            esc = true;
            break;
        }
        if (check(SelPress) || check(NextPress) || check(PrevPress)) break;
        delay(20);
    }
    (void)esc;
}

// ---------------------------------------------------------------------------
// Offline .sub decoding
// ---------------------------------------------------------------------------

void show_reading(const WeatherReading &st, const std::vector<int> &durations, const String &path) {
    String body;
    body += "File: ";
    body += base_name(path);
    body += "\n\n";
    body += st.model;
    body += "  id ";
    body += String(st.id);
    body += "  ch ";
    body += String(st.channel);
    body += "\n";
    if (st.hasTemp) {
        body += String(st.tempC, 1);
        body += "C / ";
        body += String(st.tempC * 9.0f / 5.0f + 32.0f, 1);
        body += "F\n";
    }
    if (st.hasHumidity) {
        body += String(st.humidity);
        body += "% RH\n";
    }
    if (st.hasWind) {
        body += "Wind ";
        body += String(st.windKph, 1);
        body += " km/h\n";
    }
    if (st.hasRain) {
        body += "Rain ";
        body += String(st.rainMm, 2);
        body += " mm\n";
    }
    body += st.battery ? "Battery OK\n" : "Battery LOW\n";
    body += "\npulses: ";
    body += String((int)durations.size());
    show_text_screen("Weather Decode", body);
}

void decode_sub_file(FS *fs, const String &path) {
    File probe = fs->open(path, FILE_READ);
    if (!probe) {
        show_text_screen("Weather Decode", "Cannot open file.");
        return;
    }
    const size_t size = probe.size();
    probe.close();
    if (size > 512 * 1024) {
        show_text_screen("Weather Decode", "File too large (max 512 KB).\nUse a RAW capture, not a long recording.");
        return;
    }

    const String text = read_text(fs, path);
    if (text.indexOf("Protocol: BinRAW") >= 0 || text.indexOf("Data_RAW:") >= 0) {
        show_text_screen(
            "Weather Decode",
            "This is a BinRAW key file, not a RAW pulse capture.\n\nRe-capture it in Read RAW mode to decode.");
        return;
    }

    std::vector<int> durations = weather_parse_sub(text);
    if (durations.empty()) {
        show_text_screen("Weather Decode", "No RAW_Data in this file.");
        return;
    }

    std::vector<WeatherRepeat> repeats;
    WeatherReading st;
    // File decoding should accept a single Nexus/Prologue frame (many sample
    // files contain one row), so minRepeats = 1.
    if (weather_decode(durations, repeats, st, 1)) {
        Serial.printf(
            "[Weather] file DECODED %s id=%d ch=%d temp=%.1f\n",
            st.model.c_str(),
            st.id,
            st.channel,
            st.tempC);
        show_reading(st, durations, path);
    } else {
        Serial.printf("[Weather] file no decode: %s\n", weather_diagnose(durations).c_str());
        show_text_screen("No decode", weather_diagnose(durations));
    }
}

void browse_weather_files(FS *fs, String path) {
    while (true) {
        File dir = fs->open(path);
        if (!dir) {
            displayError("Open failed", true);
            return;
        }

        struct Entry {
            String name;
            bool isDir;
        };
        std::vector<Entry> entries;
        if (path != "/") entries.push_back({"..", true});

        File e;
        while ((e = dir.openNextFile())) {
            String n = e.name();
            const int slash = n.lastIndexOf('/');
            if (slash >= 0) n = n.substring(slash + 1);
            if (n.length() && n[0] != '.') {
                if (e.isDirectory()) entries.push_back({n, true});
                else if (n.endsWith(".sub")) entries.push_back({n, false});
            }
            e.close();
        }
        dir.close();

        if (entries.empty()) {
            show_text_screen("Weather Decode", "No .sub files or folders here.");
            return;
        }

        int sel = -1;
        bool selDir = false;
        options = {};
        for (size_t i = 0; i < entries.size(); i++) {
            options.emplace_back(entries[i].name, [&sel, &selDir, i, &entries]() {
                sel = (int)i;
                selDir = entries[i].isDir;
            });
        }
        options.emplace_back("Back", []() {});
        loopOptions(options, MENU_TYPE_SUBMENU, path.c_str());

        if (sel < 0) return;
        if (selDir) {
            if (entries[sel].name == "..") {
                const int slash = path.lastIndexOf('/');
                path = slash <= 0 ? String("/") : path.substring(0, slash);
            } else {
                path = path + "/" + entries[sel].name;
            }
            continue;
        }
        decode_sub_file(fs, path + "/" + entries[sel].name);
    }
}

void decode_file_menu() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) {
        displayError("No storage", true);
        return;
    }
    browse_weather_files(fs, "/");
}

// ---------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------

void settings_menu() {
    while (true) {
        int option = 0;
        options = {
            {"Frequency: " + String(g_freq, 3) + " MHz", [&]() { option = 1; }},
            {"RX BW: " + String((int)g_bw) + " kHz",     [&]() { option = 2; }},
            {"AGC: " + String(agc_label(g_agc)),         [&]() { option = 3; }},
            {"RX preset: " + String(preset_label(g_preset)), [&]() { option = 4; }},
            {"Save captures: " + String(g_saveCaptures ? "ON" : "OFF"),
             [&]() {
                 g_saveCaptures = !g_saveCaptures;
                 settings_save();
             }},
            {"Serial debug: " + String(g_debug ? "ON" : "OFF"),
             [&]() {
                 g_debug = !g_debug;
                 settings_save();
             }},
            {"Decode .sub file", [&]() { option = 5; }},
            {"Back", [&]() { option = 0; }},
        };
        loopOptions(options, MENU_TYPE_SUBMENU, "Weather Settings");
        if (option == 0) return;

        if (option == 1) {
            int pick = -1;
            options = {
                {"433.920 MHz", [&]() { pick = 0; }},
                {"433.790 MHz (Nexus/LaCrosse)", [&]() { pick = 1; }},
                {"434.100 MHz (AcuRite 606TX)", [&]() { pick = 2; }},
                {"433.820 MHz", [&]() { pick = 3; }},
                {"433.889 MHz", [&]() { pick = 4; }},
                {"433.420 MHz", [&]() { pick = 5; }},
                {"315.000 MHz", [&]() { pick = 6; }},
                {"868.350 MHz", [&]() { pick = 7; }},
                {"915.000 MHz", [&]() { pick = 8; }},
                {"Custom (kHz)...", [&]() { pick = 9; }},
                {"Cancel", [&]() { pick = -1; }},
            };
            loopOptions(options);
            const float vals[] = {433.92f, 433.79f, 434.10f, 433.82f, 433.889f,
                                  433.42f, 315.0f,  868.35f, 915.0f};
            if (pick >= 0 && pick < 9) {
                g_freq = vals[pick];
                settings_save();
            } else if (pick == 9) {
                String s = num_keyboard(String((int)(g_freq * 1000.0f)), 6, "Frequency in kHz");
                const long khz = s.toInt();
                if (khz >= 300000L && khz <= 928000L) {
                    g_freq = (float)khz / 1000.0f;
                    settings_save();
                }
            }
        } else if (option == 2) {
            int pick = -1;
            options = {
                {"58 kHz",  [&]() { pick = 58; }},  {"135 kHz", [&]() { pick = 135; }},
                {"256 kHz", [&]() { pick = 256; }}, {"406 kHz", [&]() { pick = 406; }},
                {"Cancel",  [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick > 0) {
                g_bw = (float)pick;
                settings_save();
            }
        } else if (option == 3) {
            int pick = -1;
            options = {
                {"Default",     [&]() { pick = 0; }},
                {"AGC C7",      [&]() { pick = 1; }},
                {"AGC C3",      [&]() { pick = 2; }},
                {"Max Sensitivity", [&]() { pick = 3; }},
                {"Cancel",      [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick >= 0) {
                g_agc = pick;
                settings_save();
            }
        } else if (option == 4) {
            int pick = -1;
            options = {
                {"Flipper OOK650 (long files)", [&]() { pick = 1; }},
                {"Flipper OOK270 (short files)", [&]() { pick = 0; }},
                {"Manual BW/AGC", [&]() { pick = 2; }},
                {"Cancel", [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick >= 0) {
                g_preset = pick;
                settings_save();
            }
        } else if (option == 5) {
            decode_file_menu();
        }
    }
}

// ---------------------------------------------------------------------------
// Live listening
// ---------------------------------------------------------------------------

void merge_station(
    std::vector<WeatherReading> &stations, std::vector<int> &counts, const WeatherReading &st) {
    for (size_t i = 0; i < stations.size(); i++) {
        if (stations[i].model == st.model && stations[i].id == st.id &&
            stations[i].channel == st.channel) {
            stations[i] = st;
            counts[i]++;
            return;
        }
    }
    stations.insert(stations.begin(), st);
    counts.insert(counts.begin(), 1);
    if (stations.size() > 8) {
        stations.pop_back();
        counts.pop_back();
    }
}

void weather_live() {
    bruceConfigPins.rfFreq = g_freq;
    RfRxSession rx;
    if (!rx.begin(false, 2048)) {
        displayError("RX session failed", true);
        return;
    }
    apply_preset();

    std::vector<WeatherReading> stations;
    std::vector<int> counts;
    std::vector<WeatherRepeat> repeats;
    std::vector<int> carry;
    int attempts = 0;
    int rssi = -127;
    uint32_t lastDraw = 0;
    uint32_t lastHeartbeat = 0;
    uint32_t lastDump = 0;
    uint32_t lastDiag = 0;

    draw_weather_screen(stations, counts, rssi, attempts, true, true);

    while (!check(EscPress)) {
        if (check(SelPress)) {
            const float prevFreq = g_freq;
            const float prevBw = g_bw;
            const int prevAgc = g_agc;
            const int prevPreset = g_preset;
            settings_menu();
            if (g_freq != prevFreq || g_bw != prevBw || g_agc != prevAgc ||
                g_preset != prevPreset) {
                rx.end();
                deinitRfModule();
                bruceConfigPins.rfFreq = g_freq;
                if (!rx.begin(false, 2048)) {
                    displayError("CC1101 re-init failed", true);
                    return;
                }
                apply_preset();
            }
            draw_weather_screen(stations, counts, rssi, attempts, true, true);
        }

        if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();

        std::vector<int> chunk;
        bool updated = false;
        if (rx.poll(chunk)) {
            const uint32_t now = millis();
            attempts++;
            if (g_debug) {
                Serial.printf("[Weather] poll %u durations\n", (unsigned)chunk.size());
                if (chunk.size() >= 40 && now - lastDump > 2000) {
                    lastDump = now;
                    int pmin = 2000000, pmax = 0, nmin = 2000000, nmax = 0;
                    for (int v : chunk) {
                        if (v > 0) {
                            if (v < pmin) pmin = v;
                            if (v > pmax) pmax = v;
                        } else {
                            const int a = -v;
                            if (a < nmin) nmin = a;
                            if (a > nmax) nmax = a;
                        }
                    }
                    Serial.printf(
                        "[Weather] chunk=%u pos %d..%d neg %d..%d\n",
                        (unsigned)chunk.size(),
                        pmin,
                        pmax,
                        nmin,
                        nmax);
                    String line;
                    const size_t limit = chunk.size() < 48 ? chunk.size() : 48;
                    for (size_t i = 0; i < limit; i++) {
                        line += String(chunk[i]);
                        line += ' ';
                        if ((i + 1) % 24 == 0) {
                            Serial.printf("[Weather] d%03u: %s\n", (unsigned)(i - 23), line.c_str());
                            line = "";
                        }
                    }
                    if (line.length()) Serial.printf("[Weather] dtail: %s\n", line.c_str());
                }
            }

            // Join this capture with the tail of the previous one so a frame
            // split across RMT captures still has its full bit train.
            std::vector<int> window;
            window.reserve(carry.size() + chunk.size());
            window.insert(window.end(), carry.begin(), carry.end());
            window.insert(window.end(), chunk.begin(), chunk.end());

            WeatherReading st;
            if (weather_decode(window, repeats, st, 3)) {
                if (g_saveCaptures) save_capture(window, g_freq, st.model);
                merge_station(stations, counts, st);
                Serial.printf(
                    "[Weather] DECODED %s id=%d ch=%d temp=%.1f hum=%d batt=%d\n",
                    st.model.c_str(),
                    st.id,
                    st.channel,
                    st.tempC,
                    st.hasHumidity ? st.humidity : -1,
                    st.battery ? 1 : 0);
                updated = true;
            } else if (g_debug && now - lastDiag > 1000) {
                lastDiag = now;
                Serial.printf("[Weather] no decode: %s\n", weather_diagnose(window).c_str());
            }

            const size_t keep = 512;
            if (window.size() > keep) {
                carry.assign(window.end() - keep, window.end());
            } else {
                carry = window;
            }
        }

        const uint32_t nowMs = millis();
        if (g_debug && nowMs - lastHeartbeat > 5000) {
            Serial.printf(
                "[Weather] listening rssi=%d reads=%d stations=%u\n",
                rssi,
                attempts,
                (unsigned)stations.size());
            lastHeartbeat = nowMs;
        }
        if (updated || nowMs - lastDraw > 500) {
            draw_weather_screen(stations, counts, rssi, attempts, true, false);
            lastDraw = nowMs;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    rx.end();
    deinitRfModule();
}

} // namespace

void rf_weather_station() {
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    // Never let a full USB-CDC TX buffer block the UI/weather loop. Without
    // this, a debug build with no host reading can freeze the app mid-print.
    Serial.setTxTimeoutMs(0);
#endif
    settings_load();

    while (true) {
        int choice = 0;
        options = {
            {"Live listen", [&]() { choice = 1; }},
            {"Decode .sub file", [&]() { choice = 2; }},
            {"Radio settings", [&]() { choice = 3; }},
            {"Back", [&]() { choice = 0; }},
        };
        loopOptions(options, MENU_TYPE_SUBMENU, "Weather Station");
        if (choice == 0) return;
        if (choice == 1) weather_live();
        else if (choice == 2) decode_file_menu();
        else if (choice == 3) settings_menu();
    }
}
