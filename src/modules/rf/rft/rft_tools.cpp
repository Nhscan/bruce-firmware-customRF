// RF Tools umbrella menu + the two supporting screens that do not already
// exist in Bruce: Key Info (capture-and-explain) and Rolling Codes (capture a
// sequence, replay it in order).
#include "rft.h"

#include "../protocols/rf_decoder.h"
#include "../rf_bruteforce.h"
#include "../rf_capture_tx.h"
#include "../rf_repeater.h"
#include "../rf_scan.h"
#include "../rf_send.h"
#include "../rf_spectrum.h"
#include "../rf_subghz.h"
#include "../rf_utils.h"

#include "core/display.h"
#include "core/sd_functions.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <globals.h>

namespace {

constexpr const char *ROLL_DIR = "/BruceRF/Rolling";
constexpr size_t RFT_KEYINFO_MAX = 4096;

void apply_agc_local(int agc) {
    if (agc == 1) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    else if (agc == 2) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC3);
    else if (agc == 3) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x07);
}

bool begin_rx(RfRxSession &rx) {
    bruceConfigPins.rfFreq = rftSettings.freq;
    if (!rx.begin()) return false;
    ELECHOUSE_cc1101.setRxBW(rftSettings.bw);
    apply_agc_local(rftSettings.agc);
    return true;
}

const char *protocol_hint(const char *protocol) {
    if (!protocol) return "";
    if (!strcmp(protocol, "KeeLoq"))
        return "HCS301/HCS360-style rolling code.\nCommon on 1990s-2010s car\nremotes and gates. The\ncounter advances every press,\nso stale replays usually fail.";
    if (!strcmp(protocol, "Honda V1"))
        return "Honda keyless entry (68-bit,\n1 ms/2 ms PWM). Lock/unlock,\ntrunk and panic buttons.\nCounter is 16-bit; the encoder\nadvances it on each send.";
    if (!strcmp(protocol, "Chrysler V0"))
        return "Chrysler/Jeep/Dodge RKE.\nTwo encoded blocks, 6-bit\nrolling counter. The decoder\nvalidates the block pairing and\nreports the button from the\nencoded tail.";
    if (!strcmp(protocol, "Kia V1"))
        return "Kia/Hyundai RKE. Three\nManchester bursts, 57-bit\npayload with a 4-bit CRC and a\n12-bit counter.";
    if (!strcmp(protocol, "Fiat V2"))
        return "Fiat/Jeep keyless (112-bit\nManchester). The Hop field is\nkeyed and not derivable from\ncaptures, so replay is RAW.";
    if (!strcmp(protocol, "Princeton"))
        return "Generic fixed-code remotes,\nincluding PT2260/PT2262 clones.\nVery common on cheap gates.";
    if (!strcmp(protocol, "CAME"))
        return "CAME gate motors. Fixed code.\nCommon across Europe.";
    if (!strcmp(protocol, "Nice FLO"))
        return "Nice gate motors. Fixed code.";
    if (!strcmp(protocol, "Holtek_HT12X"))
        return "Holtek HT12X fixed-code\nremotes and sensors.";
    if (!strcmp(protocol, "Linear"))
        return "Linear/Moore-o-Matic gates.\nFixed code.";
    if (!strcmp(protocol, "Cham_Code"))
        return "Chamberlain garage doors.\nFixed 7/8/9-bit code.";
    return "No usage note for this protocol.";
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

    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setCursor(10, tftHeight - 20);
    tft.print("Press any key");

    delay(250);
    while (!check(EscPress) && !check(SelPress) && !check(NextPress) && !check(PrevPress)) {
        delay(20);
    }
}

void draw_capture_status(const char *title, const char *line, int rssi) {
    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder();
    tft.setTextSize(FP);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.setCursor(10, 26);
    tft.print(title);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setCursor(10, 48);
    tft.print(line);
    tft.setCursor(10, 64);
    tft.print("RSSI ");
    tft.print(rssi);
    tft.print(" dBm");
    tft.setCursor(10, tftHeight - 20);
    tft.print("ESC: cancel");
}

// Capture until an automotive or generic OOK/KeeLoq frame is decoded.
bool capture_any(RftFrame &rft, RfCodes &generic, bool &isAutomotive) {
    RfRxSession rx;
    if (!begin_rx(rx)) {
        displayError("CC1101 init failed", true);
        return false;
    }

    std::vector<int> history;
    history.reserve(RFT_KEYINFO_MAX);
    bool found = false;
    uint32_t lastDraw = 0;

    while (!check(EscPress)) {
        const uint32_t now = millis();
        if (now - lastDraw > 200) {
            int rssi = -127;
            if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();
            draw_capture_status("Key Info", "Listening... press the remote", rssi);
            lastDraw = now;
        }

        std::vector<int> chunk;
        if (rx.poll(chunk)) {
            if (history.size() + chunk.size() > RFT_KEYINFO_MAX) {
                const size_t drop = history.size() + chunk.size() - RFT_KEYINFO_MAX;
                history.erase(history.begin(), history.begin() + (int)drop);
            }
            history.insert(history.end(), chunk.begin(), chunk.end());

            RftFrame f;
            if (rft_decode(chunk, f) || rft_decode(history, f)) {
                rft = f;
                isAutomotive = true;
                found = true;
                break;
            }

            RfMatchKind kind = RfMatchKind::NONE;
            RfCodes code;
            rf_identify_capture(chunk, kind, code);
            if (kind != RfMatchKind::NONE) {
                generic = code;
                isAutomotive = false;
                found = true;
                break;
            }
        }
        delay(2);
    }

    rx.end();
    deinitRfModule();
    return found;
}

void rf_key_info() {
    rft_settings_load();

    RftFrame rft;
    RfCodes generic;
    bool automotive = false;
    if (!capture_any(rft, generic, automotive)) return;

    String body;
    if (automotive) {
        body += rft_summary(rft);
        body += "\n\n";
        body += protocol_hint(rft.protocolName());
    } else {
        body += generic.protocol;
        if (generic.key) {
            char buf[48];
            snprintf(
                buf,
                sizeof(buf),
                "\nKey:%08lX%08lX",
                (unsigned long)(generic.key >> 32),
                (unsigned long)(generic.key & 0xFFFFFFFFu));
            body += buf;
        }
        body += "\nBit:";
        body += String(generic.Bit);
        if (generic.serial) {
            char buf[40];
            snprintf(buf, sizeof(buf), "\nSn:%08lX Cnt:%04lX", (unsigned long)generic.serial,
                     (unsigned long)generic.cnt);
            body += buf;
        }
        body += "\n\n";
        body += protocol_hint(generic.protocol.c_str());
    }

    show_text_screen(automotive ? rft.protocolName() : generic.protocol.c_str(), body);
}

// --- Rolling Codes ---------------------------------------------------------

bool ensure_roll_dir(FS *fs) {
    if (!fs->exists("/BruceRF")) fs->mkdir("/BruceRF");
    if (!fs->exists(ROLL_DIR)) return fs->mkdir(ROLL_DIR);
    return true;
}

String read_file_text(FS *fs, const String &path) {
    File f = fs->open(path, FILE_READ);
    if (!f) return "";
    String s;
    while (f.available()) s += (char)f.read();
    f.close();
    return s;
}

bool write_file_text(FS *fs, const String &path, const String &text) {
    File f = fs->open(path, FILE_WRITE, true);
    if (!f) return false;
    f.write((const uint8_t *)text.c_str(), text.length());
    f.close();
    return true;
}

std::vector<String> rolling_files() {
    std::vector<String> out;
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return out;
    File dir = fs->open(ROLL_DIR);
    if (!dir) return out;

    File entry;
    while ((entry = dir.openNextFile())) {
        String n = entry.name();
        const int slash = n.lastIndexOf('/');
        if (slash >= 0) n = n.substring(slash + 1);
        if (!entry.isDirectory() && n.startsWith("rc_") && n.endsWith(".sub")) {
            out.push_back(String(ROLL_DIR) + "/" + n);
        }
        entry.close();
    }
    dir.close();

    for (size_t i = 0; i < out.size(); i++) {
        for (size_t j = i + 1; j < out.size(); j++) {
            if (out[j] < out[i]) {
                String t = out[i];
                out[i] = out[j];
                out[j] = t;
            }
        }
    }
    return out;
}

std::vector<int> parse_raw_line(const String &s) {
    std::vector<int> out;
    int i = 0;
    while (i < (int)s.length()) {
        while (i < (int)s.length() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r')) i++;
        if (i >= (int)s.length()) break;
        int sign = 1;
        if (s[i] == '-') {
            sign = -1;
            i++;
        } else if (s[i] == '+') {
            i++;
        }
        if (i >= (int)s.length() || s[i] < '0' || s[i] > '9') {
            while (i < (int)s.length() && s[i] != ' ') i++;
            continue;
        }
        long v = 0;
        while (i < (int)s.length() && s[i] >= '0' && s[i] <= '9') v = v * 10 + (s[i++] - '0');
        out.push_back((int)(sign * v));
    }
    return out;
}

bool rolling_capture() {
    RftFrame frame;
    std::vector<int> raw;
    RfRxSession rx;
    if (!begin_rx(rx)) {
        displayError("CC1101 init failed", true);
        return false;
    }

    std::vector<int> history;
    history.reserve(RFT_KEYINFO_MAX);
    bool got = false;
    uint32_t lastDraw = 0;

    while (!check(EscPress)) {
        const uint32_t now = millis();
        if (now - lastDraw > 200) {
            int rssi = -127;
            if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();
            draw_capture_status("Rolling Codes", "Listening... press the remote", rssi);
            lastDraw = now;
        }
        std::vector<int> chunk;
        if (rx.poll(chunk)) {
            if (history.size() + chunk.size() > RFT_KEYINFO_MAX) {
                const size_t drop = history.size() + chunk.size() - RFT_KEYINFO_MAX;
                history.erase(history.begin(), history.begin() + (int)drop);
            }
            history.insert(history.end(), chunk.begin(), chunk.end());
            RftFrame f;
            if (rft_decode(history, f)) {
                frame = f;
                raw = history;
                got = true;
                break;
            }
        }
        delay(2);
    }

    rx.end();
    deinitRfModule();
    if (!got) return false;

    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return false;
    if (!ensure_roll_dir(fs)) return false;

    uint32_t index = 1;
    String path;
    do {
        char name[32];
        snprintf(name, sizeof(name), "rc_%03lu.sub", (unsigned long)index++);
        path = String(ROLL_DIR) + "/" + name;
    } while (fs->exists(path));

    String rawData;
    bool hasCrc = false;
    uint64_t crc = 0;
    std::vector<int> indexed;
    int bits = 0, te = 0;
    rf_build_raw(raw, rawData, hasCrc, crc, indexed, bits, te);
    String text = rf_subghz_header(rftSettings.freq);
    text += "Preset: Ook650Async\n";
    text += "Protocol: RAW\n";
    text += "RAW_Data: " + rawData;
    if (!write_file_text(fs, path, text)) return false;

    if (frame.hasEncoder()) {
        const String kp = path.substring(0, path.length() - 4) + ".key";
        write_file_text(fs, kp, rft_key_text(frame, rftSettings.freq));
    }
    displaySuccess(path.substring(path.lastIndexOf('/') + 1), true);
    return true;
}

float rolling_frequency(const String &text, float fallback) {
    const int idx = text.indexOf("Frequency:");
    if (idx < 0) return fallback;
    const int nl = text.indexOf('\n', idx);
    String v = text.substring(idx + 10, nl < 0 ? text.length() : nl);
    v.trim();
    const long hz = v.toInt();
    if (hz >= 1000000L) return (float)hz / 1000000.0f;
    return fallback;
}

bool rolling_send_file(FS *fs, const String &path) {
    const String sub = read_file_text(fs, path);
    const float freq = rolling_frequency(sub, rftSettings.freq);
    const String keyPath = path.substring(0, path.length() - 4) + ".key";

    if (fs->exists(keyPath)) {
        RftFrame f;
        if (rft_parse_key(read_file_text(fs, keyPath), f)) {
            f.cnt += (uint32_t)rftSettings.counterIncr;
            std::vector<int> pulses;
            if (rft_encode(f, pulses)) {
                bool any = false;
                for (int r = 0; r < rftSettings.repeat; r++) {
                    any = rf_transmit_raw(pulses, freq) || any;
                    if (r < rftSettings.repeat - 1) delay(400);
                }
                if (any) return true;
            }
        }
    }

    const int idx = sub.indexOf("RAW_Data:");
    if (idx < 0) return false;
    const int nl = sub.indexOf('\n', idx);
    const std::vector<int> raw = parse_raw_line(sub.substring(idx + 9, nl < 0 ? sub.length() : nl));
    if (raw.empty()) return false;
    bool any = false;
    for (int r = 0; r < rftSettings.repeat; r++) {
        any = rf_transmit_raw(raw, freq) || any;
        if (r < rftSettings.repeat - 1) delay(400);
    }
    return any;
}

int g_rollingSent = 0;

void rolling_send_next() {
    auto files = rolling_files();
    if (files.empty()) {
        displayInfo("No rolling captures yet", true);
        return;
    }

    rft_settings_load();
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;

    if (g_rollingSent < 0 || g_rollingSent >= (int)files.size()) g_rollingSent = 0;
    if (!rolling_send_file(fs, files[g_rollingSent])) {
        displayError("Send failed", true);
        return;
    }
    g_rollingSent++;
    displaySuccess("Sent " + String(g_rollingSent) + "/" + String(files.size()), true);
}

void rolling_send_all() {
    auto files = rolling_files();
    if (files.empty()) {
        displayInfo("No rolling captures yet", true);
        return;
    }

    rft_settings_load();
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;

    for (size_t i = 0; i < files.size(); i++) {
        if (!rolling_send_file(fs, files[i])) {
            displayError("Send failed at " + String((int)i + 1), true);
            return;
        }
        delay(400);
    }
    g_rollingSent = (int)files.size();
    displaySuccess("Sent whole set", true);
}

void rolling_delete() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    auto files = rolling_files();
    for (const auto &p : files) {
        if (fs->exists(p)) fs->remove(p);
        const String kp = p.substring(0, p.length() - 4) + ".key";
        if (fs->exists(kp)) fs->remove(kp);
    }
    displaySuccess("Rolling set cleared", true);
}

void rf_rolling_codes() {
    int choice = 0;
    while (true) {
        const int count = (int)rolling_files().size();
        int nextShown = (count > 0 && g_rollingSent < count) ? g_rollingSent + 1 : (count > 0 ? 1 : 0);
        options = {
            {"Capture code", [&]() { choice = 1; }},
            {"Send next (" + String(nextShown) + "/" + String(count) + ")", [&]() { choice = 2; }},
            {"Send whole set", [&]() { choice = 3; }},
            {"Delete set (" + String(count) + ")", [&]() { choice = 4; }},
            {"Back", [&]() { choice = 0; }},
        };
        loopOptions(options, MENU_TYPE_SUBMENU, "Rolling Codes");
        if (choice == 0) return;
        if (choice == 1) {
            if (rolling_capture()) g_rollingSent = 0;
        } else if (choice == 2) {
            rolling_send_next();
        } else if (choice == 3) {
            rolling_send_all();
        } else if (choice == 4) {
            rolling_delete();
            g_rollingSent = 0;
        }
    }
}

} // namespace

void rf_tools_menu() {
    rft_settings_load();
    options = {
        {"Car Keys",           rf_car_keys             },
        {"Capture (Read)",     []() { RFScan scan; }   },
        {"Capture RAW",        []() { RFScan scan(true); }},
        {"Repeater",           rf_repeater             },
        {"Bruteforce",         rf_bruteforce           },
        {"Frequency Analyzer", rf_frequency_analyzer   },
        {"Key Info",           rf_key_info             },
        {"Rolling Codes",      rf_rolling_codes        },
        {"Add Manually",       rf_add_manually         },
        {"Signal Meter",       rf_CC1101_rssi          },
        {"Saved Signals",      sendCustomRF            },
        {"Radio Settings",     rft_radio_settings_menu },
        {"Back",               []() {}                 },
    };
    loopOptions(options, MENU_TYPE_SUBMENU, "RF Tools");
}
