// Car Keys -- continuous automotive capture and replay for Bruce.
//
// Mirrors the Flipper RF Tools "Car Keys" flow:
//   - listen continuously with a rolling edge buffer (noise overwrites the
//     oldest edges, so a fob press is caught on the first try),
//   - decode Honda V1 / Chrysler V0 / Kia V1 / Fiat V2 locally,
//   - log every press and save a RAW .sub plus a .key companion,
//   - replay either the encoded next frame or the RAW capture.
#include "rft.h"

#include "../rf_capture_tx.h"
#include "../rf_send.h"
#include "../rf_utils.h"
#include "../protocols/rf_decoder.h"

#include "core/display.h"
#include "core/mykeyboard.h"
#include "core/sd_functions.h"
#include "core/utils.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <globals.h>

#include <stdio.h>
#include <string.h>

namespace {

constexpr const char *RFT_ROOT = "/BruceRF";
constexpr const char *CAR_DIR = "/BruceRF/CarKeys";
constexpr const char *KEY_DIR = "/BruceRF/Keys";
constexpr const char *CONF_PATH = "/BruceRF/rft.conf";
constexpr size_t RFT_HISTORY_MAX = 4096;
constexpr uint32_t RFT_ARM_SETTLE_MS = 400;

// ---------------------------------------------------------------------------
// Settings (shared with the RF Tools menu)
// ---------------------------------------------------------------------------

} // namespace

RftSettings rftSettings;

void rft_settings_load() {
    rftSettings.freq = 433.92f;
    rftSettings.bw = 135.0f;
    rftSettings.agc = 3;
    rftSettings.repeat = 3;
    rftSettings.counterIncr = 1;
    rftSettings.autoSave = true;

    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    File f = fs->open(CONF_PATH, FILE_READ);
    if (!f) return;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (!line.length()) continue;
        const int eq = line.indexOf('=');
        if (eq < 0) continue;
        String key = line.substring(0, eq);
        String val = line.substring(eq + 1);
        key.trim();
        val.trim();
        if (key == "freq") rftSettings.freq = val.toFloat();
        else if (key == "bw") rftSettings.bw = val.toFloat();
        else if (key == "agc") rftSettings.agc = val.toInt();
        else if (key == "repeat") rftSettings.repeat = val.toInt();
        else if (key == "counter") rftSettings.counterIncr = val.toInt();
        else if (key == "autosave") rftSettings.autoSave = val.toInt() != 0;
    }
    f.close();
}

void rft_settings_save() {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    if (!fs->exists(RFT_ROOT)) fs->mkdir(RFT_ROOT);
    File f = fs->open(CONF_PATH, FILE_WRITE, true);
    if (!f) return;
    String out;
    out += "freq=" + String(rftSettings.freq, 3) + "\n";
    out += "bw=" + String((int)rftSettings.bw) + "\n";
    out += "agc=" + String(rftSettings.agc) + "\n";
    out += "repeat=" + String(rftSettings.repeat) + "\n";
    out += "counter=" + String(rftSettings.counterIncr) + "\n";
    out += "autosave=" + String(rftSettings.autoSave ? 1 : 0) + "\n";
    f.write((const uint8_t *)out.c_str(), out.length());
    f.close();
}

namespace {

const char *agc_label(int agc) {
    switch (agc) {
    case 1: return "AGC C7";
    case 2: return "AGC C3";
    case 3: return "Max Sensitivity";
    default: return "Default";
    }
}

void apply_agc(int agc) {
    if (agc == 1) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC7);
    else if (agc == 2) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0xC3);
    else if (agc == 3) ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x07);
}

bool ensure_dir(FS *fs, const String &path) {
    if (fs->exists(path)) return true;
    return fs->mkdir(path);
}

bool ensure_tree(FS *fs) {
    if (!ensure_dir(fs, RFT_ROOT)) return false;
    if (!ensure_dir(fs, CAR_DIR)) return false;
    return ensure_dir(fs, KEY_DIR);
}

bool write_text(FS *fs, const String &path, const String &text) {
    File f = fs->open(path, FILE_WRITE, true);
    if (!f) return false;
    f.write((const uint8_t *)text.c_str(), text.length());
    f.close();
    return true;
}

String read_text(FS *fs, const String &path) {
    File f = fs->open(path, FILE_READ);
    if (!f) return "";
    String s;
    while (f.available()) s += (char)f.read();
    f.close();
    return s;
}

String proto_compact(const RftFrame &f) {
    switch (f.proto) {
    case RftProto::HondaV1: return "HondaV1";
    case RftProto::ChryslerV0: return "ChryslerV0";
    case RftProto::KiaV1: return "KiaV1";
    case RftProto::FiatV2: return "FiatV2";
    default: return "Unknown";
    }
}

String base_name(const String &path) {
    const int slash = path.lastIndexOf('/');
    return slash >= 0 ? path.substring(slash + 1) : path;
}

uint32_t next_capture_index(FS *fs) {
    File dir = fs->open(CAR_DIR);
    if (!dir) return 1;
    uint32_t highest = 0;
    File entry;
    while ((entry = dir.openNextFile())) {
        String n = base_name(entry.name());
        if (n.startsWith("ck_")) {
            const uint32_t idx = (uint32_t)n.substring(3).toInt();
            if (idx > highest) highest = idx;
        }
        entry.close();
    }
    dir.close();
    return highest + 1;
}

std::vector<int> parse_raw_data(const String &s) {
    std::vector<int> out;
    int i = 0;
    const int len = s.length();
    while (i < len) {
        while (i < len && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r')) i++;
        if (i >= len) break;
        int sign = 1;
        if (s[i] == '-') {
            sign = -1;
            i++;
        } else if (s[i] == '+') {
            i++;
        }
        if (i >= len || s[i] < '0' || s[i] > '9') {
            while (i < len && s[i] != ' ') i++;
            continue;
        }
        long v = 0;
        while (i < len && s[i] >= '0' && s[i] <= '9') {
            v = v * 10 + (s[i] - '0');
            i++;
        }
        out.push_back((int)(sign * v));
    }
    return out;
}

String raw_sub_text(const std::vector<int> &raw, float freq) {
    String rawData;
    bool hasCrc = false;
    uint64_t crc = 0;
    std::vector<int> indexed;
    int bits = 0, te = 0;
    rf_build_raw(raw, rawData, hasCrc, crc, indexed, bits, te);

    String out = rf_subghz_header(freq);
    out += "Preset: Ook650Async\n";
    out += "Protocol: RAW\n";
    out += "RAW_Data: " + rawData;
    return out;
}

String key_path_for_sub(const String &subPath) {
    if (subPath.endsWith(".sub")) return subPath.substring(0, subPath.length() - 4) + ".key";
    return subPath + ".key";
}

String save_capture(const RftFrame &frame, const std::vector<int> &raw, String *outName = nullptr) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return "";
    if (!ensure_tree(fs)) return "";

    char name[72];
    snprintf(
        name,
        sizeof(name),
        "ck_%03lu_%s_Btn%02X_Cnt%04lX",
        (unsigned long)next_capture_index(fs),
        proto_compact(frame).c_str(),
        (unsigned)frame.btn,
        (unsigned long)frame.cnt);
    const String path = String(CAR_DIR) + "/" + name + ".sub";
    const String text = raw_sub_text(raw, rftSettings.freq);
    if (!write_text(fs, path, text)) return "";

    if (frame.hasEncoder()) {
        write_text(fs, key_path_for_sub(path), rft_key_text(frame, rftSettings.freq));
    }
    if (outName) *outName = String(name);
    return path;
}

int proto_index(RftProto p) {
    switch (p) {
    case RftProto::HondaV1: return 0;
    case RftProto::ChryslerV0: return 1;
    case RftProto::KiaV1: return 2;
    case RftProto::FiatV2: return 3;
    default: return -1;
    }
}

uint32_t g_lastCnt[4] = {0};
bool g_haveLastCnt[4] = {false};

void log_decode(const RftFrame &frame, const String &savedPath) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    if (!ensure_tree(fs)) return;

    const int pi = proto_index(frame.proto);
    bool havePrev = false;
    int32_t delta = 0;
    if (pi >= 0) {
        havePrev = g_haveLastCnt[pi];
        if (havePrev) delta = (int32_t)(frame.cnt - g_lastCnt[pi]);
        g_lastCnt[pi] = frame.cnt;
        g_haveLastCnt[pi] = true;
    }

    File f = fs->open(String(CAR_DIR) + "/car_log.txt", FILE_APPEND);
    if (!f) f = fs->open(String(CAR_DIR) + "/car_log.txt", FILE_WRITE, true);
    if (!f) return;

    char line[200];
    char dtmp[24] = "";
    if (havePrev) snprintf(dtmp, sizeof(dtmp), " d:%+ld", (long)delta);

    if (frame.proto == RftProto::FiatV2) {
        snprintf(
            line,
            sizeof(line),
            "%lu ms  %s  Sn:%08lX Btn:%02X Cnt:%04lX Hop:%08lX%s  %s\n",
            (unsigned long)millis(),
            frame.protocolName(),
            (unsigned long)frame.serial,
            (unsigned)frame.btn,
            (unsigned long)frame.cnt,
            (unsigned long)frame.hop,
            dtmp,
            base_name(savedPath).c_str());
    } else {
        snprintf(
            line,
            sizeof(line),
            "%lu ms  %s  Sn:%08lX Btn:%02X Cnt:%04lX%s  %s\n",
            (unsigned long)millis(),
            frame.protocolName(),
            (unsigned long)frame.serial,
            (unsigned)frame.btn,
            (unsigned long)frame.cnt,
            dtmp,
            base_name(savedPath).c_str());
    }
    f.write((const uint8_t *)line, strlen(line));
    f.close();
}

bool reinit_rx(RfRxSession &rx, uint32_t *armedAt = nullptr) {
    bruceConfigPins.rfFreq = rftSettings.freq;
    if (!rx.begin(false, 2048)) return false;
    ELECHOUSE_cc1101.setRxBW(rftSettings.bw);
    apply_agc(rftSettings.agc);
    if (armedAt) *armedAt = millis();
    return true;
}

bool send_pulses(const std::vector<int> &pulses, int repeat, float freq) {
    if (pulses.empty()) return false;
    bool ok = false;
    for (int i = 0; i < repeat; i++) {
        ok = rf_transmit_raw(pulses, freq) || ok;
        if (i < repeat - 1) delay(400);
    }
    return ok;
}

bool send_decoded(RftFrame frame, bool bumpCounter, float freq) {
    if (!frame.hasEncoder()) return false;
    if (bumpCounter) {
        frame.cnt += (uint32_t)rftSettings.counterIncr;
        if (frame.proto == RftProto::HondaV1) frame.cnt &= 0xFFFF;
        else if (frame.proto == RftProto::KiaV1) frame.cnt &= 0xFFF;
        else if (frame.proto == RftProto::ChryslerV0) frame.cnt &= 0x3F;
    }
    std::vector<int> pulses;
    if (!rft_encode(frame, pulses)) return false;
    return send_pulses(pulses, rftSettings.repeat, freq);
}

float freq_from_sub_text(const String &text, float fallback) {
    const int idx = text.indexOf("Frequency:");
    if (idx < 0) return fallback;
    const int nl = text.indexOf('\n', idx);
    String v = text.substring(idx + 10, nl < 0 ? text.length() : nl);
    v.trim();
    const long hz = v.toInt();
    if (hz >= 1000000L) return (float)hz / 1000000.0f;
    return fallback;
}

String next_free_name(FS *fs, const char *prefix, const char *ext) {
    uint32_t idx = 1;
    String path;
    do {
        char name[48];
        snprintf(name, sizeof(name), "%s%03lu.%s", prefix, (unsigned long)idx++, ext);
        path = String(KEY_DIR) + "/" + name;
    } while (fs->exists(path));
    return path;
}

String archive_capture(const RftFrame &frame, const std::vector<int> &raw) {
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return "";
    if (!ensure_tree(fs)) return "";

    char prefix[40];
    snprintf(prefix, sizeof(prefix), "%s_Btn%02X_", proto_compact(frame).c_str(), (unsigned)frame.btn);
    String path = next_free_name(fs, prefix, "sub");
    if (!write_text(fs, path, raw_sub_text(raw, rftSettings.freq))) return "";

    if (frame.hasEncoder()) {
        const String kp = path.substring(0, path.length() - 4) + ".key";
        write_text(fs, kp, rft_key_text(frame, rftSettings.freq));
    }
    return path;
}

void draw_listen_header(bool fullRedraw) {
    if (fullRedraw) {
        tft.fillScreen(bruceConfig.bgColor);
        drawMainBorder();
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.setCursor(10, 26);
        tft.print("Car Keys");
    }

    tft.setTextSize(FP);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.fillRect(10, 40, tftWidth - 20, 10, bruceConfig.bgColor);
    tft.setCursor(10, 40);
    tft.print(rftSettings.freq, 3);
    tft.print(" MHz");
}

void draw_listening(int rssi, size_t edges, bool fullRedraw) {
    draw_listen_header(fullRedraw);

    tft.fillRect(10, 54, tftWidth - 20, 12, bruceConfig.bgColor);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setCursor(10, 54);
    tft.print("Listening... press anytime");

    tft.fillRect(10, 72, tftWidth - 20, 12, bruceConfig.bgColor);
    tft.setCursor(10, 72);
    tft.print("RSSI ");
    tft.print(rssi);
    tft.print(" dBm   ");
    tft.print((unsigned int)edges);
    tft.print(" edges");

    if (fullRedraw) {
        tft.setCursor(10, tftHeight - 24);
        tft.print("ESC: back");
    }
}

void draw_result(const RftFrame &frame, const String &saved, bool fullRedraw) {
    if (fullRedraw) {
        tft.fillScreen(bruceConfig.bgColor);
        drawMainBorder();
        tft.setTextSize(FP);
        tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
        tft.setCursor(10, 26);
        tft.print("Car Keys");
    }

    tft.setTextSize(FP);
    tft.fillRect(10, 40, tftWidth - 20, 100, bruceConfig.bgColor);
    tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
    tft.setCursor(10, 40);
    tft.print(frame.protocolName());
    tft.setCursor(10, 56);
    tft.print("Button: ");
    tft.print(frame.buttonName());

    char buf[64];
    tft.setCursor(10, 72);
    snprintf(buf, sizeof(buf), "Sn: %08lX", (unsigned long)frame.serial);
    tft.print(buf);

    tft.setCursor(10, 88);
    if (frame.proto == RftProto::FiatV2) {
        snprintf(buf, sizeof(buf), "Btn: %02X  Hop: %08lX", (unsigned)frame.btn,
                 (unsigned long)frame.hop);
    } else {
        snprintf(buf, sizeof(buf), "Btn: %02X  Cnt: %04lX", (unsigned)frame.btn,
                 (unsigned long)frame.cnt);
    }
    tft.print(buf);

    tft.setCursor(10, 104);
    if (frame.proto == RftProto::FiatV2) {
        snprintf(buf, sizeof(buf), "Cnt: %04lX", (unsigned long)frame.cnt);
    } else {
        snprintf(buf, sizeof(buf), "Key: %08lX%08lX", (unsigned long)(frame.key >> 32),
                 (unsigned long)(frame.key & 0xFFFFFFFFu));
    }
    tft.print(buf);

    tft.setCursor(10, 120);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    if (frame.hasEncoder()) {
        tft.print("OK: send decoded");
    } else {
        tft.print("No encoder - raw only");
    }
    tft.setCursor(10, 136);
    tft.print("SEL: actions   ESC: back");

    if (saved.length()) {
        tft.setCursor(10, tftHeight - 24);
        tft.setTextColor(getColorVariation(bruceConfig.priColor), bruceConfig.bgColor);
        tft.print(base_name(saved));
    }
}

struct ReplayEntry {
    String path;
    String label;
};

std::vector<ReplayEntry> collect_replays() {
    std::vector<ReplayEntry> out;
    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return out;
    File dir = fs->open(CAR_DIR);
    if (!dir) return out;

    File entry;
    while ((entry = dir.openNextFile())) {
        const String n = base_name(entry.name());
        if (!entry.isDirectory() && n.startsWith("ck_") && n.endsWith(".sub")) {
            ReplayEntry e;
            e.label = n;
            e.path = String(CAR_DIR) + "/" + n;
            out.push_back(e);
        }
        entry.close();
    }
    dir.close();

    // ck_NNN prefix sorts naturally, but directory order is not guaranteed.
    for (size_t i = 0; i < out.size(); i++) {
        for (size_t j = i + 1; j < out.size(); j++) {
            if (out[j].label < out[i].label) {
                ReplayEntry t = out[i];
                out[i] = out[j];
                out[j] = t;
            }
        }
    }
    return out;
}

void replay_delete_all() {
    int choice = 0;
    options = {
        {"Delete all captures", [&]() { choice = 1; }},
        {"Cancel", [&]() { choice = 0; }},
    };
    loopOptions(options, MENU_TYPE_REGULAR, "Delete replay list?");
    if (choice != 1) return;

    FS *fs = nullptr;
    if (!getFsStorage(fs) || fs == nullptr) return;
    File dir = fs->open(CAR_DIR);
    if (!dir) return;

    std::vector<String> victims;
    File entry;
    while ((entry = dir.openNextFile())) {
        const String n = base_name(entry.name());
        if (!entry.isDirectory() && n.startsWith("ck_") && n.endsWith(".sub")) {
            victims.push_back(String(CAR_DIR) + "/" + n);
            victims.push_back(String(CAR_DIR) + "/" + n.substring(0, n.length() - 4) + ".key");
        }
        entry.close();
    }
    dir.close();
    for (const auto &p : victims) {
        if (fs->exists(p)) fs->remove(p);
    }
    displaySuccess("Replay list cleared", true);
}

void replay_list() {
    auto entries = collect_replays();
    if (entries.empty()) {
        displayInfo("Replay list is empty", true);
        return;
    }

    while (true) {
        int selected = -1;
        options = {};
        for (size_t i = 0; i < entries.size(); i++) {
            const int idx = (int)i;
            options.emplace_back(entries[i].label, [&selected, idx]() { selected = idx; });
        }
        options.emplace_back("Back", [&selected]() { selected = -1; });
        loopOptions(options, MENU_TYPE_SUBMENU, "Replay list");

        if (selected < 0) return;

        const ReplayEntry &e = entries[selected];
        const String keyPath = key_path_for_sub(e.path);
        FS *fs = nullptr;
        if (!getFsStorage(fs) || fs == nullptr) return;

        int action = 0;
        options = {
            {"Send", [&]() { action = 1; }},
            {"Back", [&]() { action = 0; }},
        };
        loopOptions(options, MENU_TYPE_REGULAR, e.label.c_str());
        if (action != 1) continue;

        bool sent = false;
        const String subText = read_text(fs, e.path);
        const float captureFreq = freq_from_sub_text(subText, rftSettings.freq);
        if (fs->exists(keyPath)) {
            RftFrame f;
            if (rft_parse_key(read_text(fs, keyPath), f)) {
                sent = send_decoded(f, true, captureFreq);
            }
        }
        if (!sent) {
            const String text = subText;
            const int idx = text.indexOf("RAW_Data:");
            if (idx >= 0) {
                const int nl = text.indexOf('\n', idx);
                const String line = text.substring(idx + 9, nl < 0 ? text.length() : nl);
                const std::vector<int> raw = parse_raw_data(line);
                sent = send_pulses(raw, rftSettings.repeat, captureFreq);
            }
        }
        displaySuccess(sent ? "Sent" : "Send failed", true);
    }
}

// Returns true when the user chose "listen again".
bool car_listen_once() {
    tft.fillScreen(bruceConfig.bgColor);
    drawMainBorder();
    draw_listening(-127, 0, true);

    RfRxSession rx;
    uint32_t armedAt = 0;
    if (!reinit_rx(rx, &armedAt)) {
        displayError("CC1101 init failed", true);
        return false;
    }

    std::vector<int> history;
    history.reserve(RFT_HISTORY_MAX);

    bool got = false;
    RftFrame frame;
    std::vector<int> rawForFrame;
    uint32_t lastDraw = 0;

    while (!check(EscPress)) {
        const uint32_t now = millis();
        if (now - lastDraw > 150) {
            int rssi = -127;
            if (bruceConfigPins.rfModule == CC1101_SPI_MODULE) rssi = ELECHOUSE_cc1101.getRssi();
            draw_listening(rssi, history.size(), false);
            lastDraw = now;
        }

        std::vector<int> chunk;
        if (rx.poll(chunk)) {
            if (millis() - armedAt < RFT_ARM_SETTLE_MS) continue;

            if (history.size() + chunk.size() > RFT_HISTORY_MAX) {
                const size_t drop = history.size() + chunk.size() - RFT_HISTORY_MAX;
                history.erase(history.begin(), history.begin() + (int)drop);
            }
            history.insert(history.end(), chunk.begin(), chunk.end());

            RftFrame f;
            if (rft_decode(history, f)) {
                got = true;
                frame = f;
                rawForFrame = history;
                break;
            }
        }
        delay(2);
    }

    rx.end();
    deinitRfModule();
    if (!got) return false;

    wakeUpScreen();
    String saved;
    if (rftSettings.autoSave) saved = save_capture(frame, rawForFrame);
    log_decode(frame, saved);

    draw_result(frame, saved, true);

    // Let the user actually read the decode before the action menu covers it.
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
    if (esc) return false;

    bool listenAgain = false;
    options = {};
    if (frame.hasEncoder()) {
        options.emplace_back(
            "Send decoded (Cnt+" + String(rftSettings.counterIncr) + ")",
            [&]() {
                if (send_decoded(frame, true, rftSettings.freq)) displaySuccess("Decoded frame sent", true);
                else displayError("Send failed", true);
            });
    }
    options.emplace_back("Send RAW capture", [&]() {
        if (send_pulses(rawForFrame, rftSettings.repeat, rftSettings.freq)) displaySuccess("RAW sent", true);
        else displayError("Send failed", true);
    });
    options.emplace_back("Save to SD", [&]() {
        const String p = save_capture(frame, rawForFrame);
        if (p.length()) displaySuccess(base_name(p), true);
        else displayError("Save failed", true);
    });
    options.emplace_back("Archive to Keys/", [&]() {
        const String p = archive_capture(frame, rawForFrame);
        if (p.length()) displaySuccess(base_name(p), true);
        else displayError("Archive failed", true);
    });
    options.emplace_back("Listen again", [&]() { listenAgain = true; });
    options.emplace_back("Back", [&]() { listenAgain = false; });
    loopOptions(options, MENU_TYPE_SUBMENU, frame.protocolName());

    return listenAgain;
}

} // namespace

void rft_radio_settings_menu() {
    rft_settings_load();

    int option = 0;
    while (true) {
        options = {
            {"Frequency: " + String(rftSettings.freq, 3) + " MHz", [&]() { option = 1; }},
            {"RX BW: " + String((int)rftSettings.bw) + " kHz",     [&]() { option = 2; }},
            {"AGC: " + String(agc_label(rftSettings.agc)),         [&]() { option = 3; }},
            {"Repeat: " + String(rftSettings.repeat),              [&]() { option = 4; }},
            {"Counter incr: " + String(rftSettings.counterIncr),   [&]() { option = 5; }},
            {"Auto-save captures: " + String(rftSettings.autoSave ? "ON" : "OFF"),
             [&]() {
                 rftSettings.autoSave = !rftSettings.autoSave;
                 rft_settings_save();
             }},
            {"Back", [&]() { option = 0; }},
        };
        loopOptions(options, MENU_TYPE_SUBMENU, "Radio Settings");
        if (option == 0) break;

        if (option == 1) {
            int pick = -1;
            options = {
                {"433.920 MHz", [&]() { pick = 0; }}, {"433.889 MHz (Fiat/Jeep)", [&]() { pick = 1; }},
                {"433.820 MHz", [&]() { pick = 2; }}, {"315.000 MHz", [&]() { pick = 3; }},
                {"868.350 MHz", [&]() { pick = 4; }}, {"915.000 MHz", [&]() { pick = 5; }},
                {"Custom (kHz)...", [&]() { pick = 6; }}, {"Cancel", [&]() { pick = -1; }},
            };
            loopOptions(options);
            const float vals[] = {433.92f, 433.889f, 433.82f, 315.0f, 868.35f, 915.0f};
            if (pick >= 0 && pick < 6) {
                rftSettings.freq = vals[pick];
                rft_settings_save();
            } else if (pick == 6) {
                String s = num_keyboard(String((int)(rftSettings.freq * 1000.0f)), 6, "Frequency in kHz");
                const long khz = s.toInt();
                if (khz >= 300000L && khz <= 928000L) {
                    rftSettings.freq = (float)khz / 1000.0f;
                    rft_settings_save();
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
                rftSettings.bw = (float)pick;
                rft_settings_save();
            }
        } else if (option == 3) {
            int pick = -1;
            options = {
                {"Default",     [&]() { pick = 0; }}, {"AGC C7", [&]() { pick = 1; }},
                {"AGC C3",      [&]() { pick = 2; }}, {"Max Sensitivity", [&]() { pick = 3; }},
                {"Cancel",      [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick >= 0) {
                rftSettings.agc = pick;
                rft_settings_save();
            }
        } else if (option == 4) {
            int pick = -1;
            options = {
                {"1",  [&]() { pick = 1; }},  {"2", [&]() { pick = 2; }},
                {"3",  [&]() { pick = 3; }},  {"5", [&]() { pick = 5; }},
                {"10", [&]() { pick = 10; }}, {"Cancel", [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick > 0) {
                rftSettings.repeat = pick;
                rft_settings_save();
            }
        } else if (option == 5) {
            int pick = -1;
            options = {
                {"1", [&]() { pick = 1; }}, {"2", [&]() { pick = 2; }},
                {"3", [&]() { pick = 3; }}, {"5", [&]() { pick = 5; }},
                {"Cancel", [&]() { pick = -1; }},
            };
            loopOptions(options);
            if (pick > 0) {
                rftSettings.counterIncr = pick;
                rft_settings_save();
            }
        }
    }
}

void rf_car_keys() {
    rft_settings_load();

    while (true) {
        int choice = 0;
        const int replayCount = (int)collect_replays().size();
        options = {
            {"Listen / Capture", [&]() { choice = 1; }},
            {"Replay list (" + String(replayCount) + ")", [&]() { choice = 2; }},
            {"Delete replay list", [&]() { choice = 3; }},
            {"Radio settings", [&]() { choice = 4; }},
            {"Back", [&]() { choice = 0; }},
        };
        loopOptions(options, MENU_TYPE_SUBMENU, "Car Keys");

        if (choice == 0) return;
        if (choice == 1) {
            while (car_listen_once()) {}
        } else if (choice == 2) {
            replay_list();
        } else if (choice == 3) {
            replay_delete_all();
        } else if (choice == 4) {
            rft_radio_settings_menu();
        }
    }
}
