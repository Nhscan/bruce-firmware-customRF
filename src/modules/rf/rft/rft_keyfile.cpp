// Flipper-compatible .key snapshot text for the automotive protocols.
// Field names follow the ProtoPirate/Flipper conventions (Serial/Btn/Cnt,
// Key as space-separated hex bytes, Chrysler Key_2 + Plain_A/Plain_B).
#include "rft.h"

#include <string.h>

namespace {

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// "11 22 33 ..." -> bytes. Also accepts contiguous hex.
int parseHexBytes(const String &s, uint8_t *out, int maxLen) {
    int n = 0;
    int hi = -1;
    for (size_t i = 0; i < s.length() && n < maxLen; i++) {
        const char c = s[i];
        if (c == ' ' || c == '\t' || c == '-' || c == ':') continue;
        const int v = hexNibble(c);
        if (v < 0) return n;
        if (hi < 0) {
            hi = v;
        } else {
            out[n++] = (uint8_t)((hi << 4) | v);
            hi = -1;
        }
    }
    return n;
}

String hexBytes(const uint8_t *data, int len) {
    String s;
    char buf[4];
    for (int i = 0; i < len; i++) {
        snprintf(buf, sizeof(buf), "%02X", data[i]);
        if (i) s += ' ';
        s += buf;
    }
    return s;
}

String fieldValue(const String &line) {
    const int idx = line.indexOf(':');
    if (idx < 0) return "";
    String v = line.substring(idx + 1);
    v.trim();
    return v;
}

uint32_t parseU32(const String &s) {
    // Accept hex with a 0x prefix so hand-written files are forgiving.
    if (s.startsWith("0x") || s.startsWith("0X")) return (uint32_t)strtoul(s.c_str(), nullptr, 16);
    return (uint32_t)strtoul(s.c_str(), nullptr, 10);
}

void appendU32Line(String &out, const char *name, uint32_t value) {
    out += name;
    out += ": ";
    out += String((unsigned long)value);
    out += "\n";
}

} // namespace

String rft_key_text(const RftFrame &frame, float freqMhz) {
    String out;
    out.reserve(700);
    out += "Filetype: Flipper SubGhz Key File\n";
    out += "Version: 1\n";
    appendU32Line(out, "Frequency", (uint32_t)((double)freqMhz * 1000.0 + 0.5) * 1000);
    out += "Preset: FuriHalSubGhzPresetOok650Async\n";
    out += "Protocol: ";
    out += frame.protocolName();
    out += "\n";
    appendU32Line(out, "Bit", frame.bits);
    appendU32Line(out, "TE", (uint32_t)frame.te);
    appendU32Line(out, "Serial", frame.serial);
    appendU32Line(out, "Btn", frame.btn);
    appendU32Line(out, "Cnt", frame.cnt);

    uint8_t keyBytes[8];
    for (int i = 0; i < 8; i++) keyBytes[i] = (uint8_t)((frame.key >> ((7 - i) * 8)) & 0xFF);
    out += "Key: ";
    out += hexBytes(keyBytes, 8);
    out += "\n";

    if (frame.proto == RftProto::ChryslerV0) {
        const uint8_t k2[2] = {(uint8_t)(frame.key2 >> 8), (uint8_t)(frame.key2 & 0xFF)};
        out += "Key_2: ";
        out += hexBytes(k2, 2);
        out += "\n";
        if (frame.plainAPresent) {
            out += "Plain_A: ";
            out += hexBytes(frame.plainA, 9);
            out += "\n";
        }
        if (frame.plainBPresent) {
            out += "Plain_B: ";
            out += hexBytes(frame.plainB, 9);
            out += "\n";
        }
        appendU32Line(out, "Seed", frame.seed);
        appendU32Line(out, "Header", frame.header);
    }

    if (frame.proto == RftProto::FiatV2) {
        appendU32Line(out, "Hop", frame.hop);
        out += "Raw: ";
        out += hexBytes(frame.raw, frame.rawLen ? frame.rawLen : 14);
        out += "\n";
    }

    if (frame.crc) appendU32Line(out, "Crc", frame.crc);
    return out;
}

bool rft_parse_key(const String &text, RftFrame &frame) {
    RftFrame f;
    String protocol;

    int start = 0;
    while (start < (int)text.length()) {
        int end = text.indexOf('\n', start);
        if (end < 0) end = text.length();
        String line = text.substring(start, end);
        start = end + 1;
        if (line.endsWith("\r")) line.remove(line.length() - 1);
        if (!line.length()) continue;

        String v = fieldValue(line);
        if (line.startsWith("Protocol:")) protocol = v;
        else if (line.startsWith("Bit:")) f.bits = (uint8_t)parseU32(v);
        else if (line.startsWith("TE:")) f.te = (int)parseU32(v);
        else if (line.startsWith("Serial:")) f.serial = parseU32(v);
        else if (line.startsWith("Btn:") || line.startsWith("Button:")) f.btn = parseU32(v);
        else if (line.startsWith("Cnt:") || line.startsWith("Counter:")) f.cnt = parseU32(v);
        else if (line.startsWith("Hop:")) f.hop = parseU32(v);
        else if (line.startsWith("Seed:")) f.seed = (uint8_t)parseU32(v);
        else if (line.startsWith("Header:")) f.header = (uint8_t)parseU32(v);
        else if (line.startsWith("Crc:")) f.crc = (uint8_t)parseU32(v);
        else if (line.startsWith("Key_2:")) {
            uint8_t b[2] = {0};
            if (parseHexBytes(v, b, 2) == 2) f.key2 = (uint16_t)((b[0] << 8) | b[1]);
        } else if (line.startsWith("Key:")) {
            uint8_t b[8] = {0};
            const int n = parseHexBytes(v, b, 8);
            if (n > 0) {
                f.key = 0;
                for (int i = 0; i < n; i++) f.key = (f.key << 8) | b[i];
            }
        } else if (line.startsWith("Plain_A:")) {
            f.plainAPresent = parseHexBytes(v, f.plainA, 9) == 9;
        } else if (line.startsWith("Plain_B:")) {
            f.plainBPresent = parseHexBytes(v, f.plainB, 9) == 9;
        } else if (line.startsWith("Raw:")) {
            f.rawLen = (uint8_t)parseHexBytes(v, f.raw, 14);
        }
    }

    if (protocol.indexOf("Honda") >= 0) f.proto = RftProto::HondaV1;
    else if (protocol.indexOf("Chrysler") >= 0) f.proto = RftProto::ChryslerV0;
    else if (protocol.indexOf("Kia") >= 0) f.proto = RftProto::KiaV1;
    else if (protocol.indexOf("Fiat") >= 0) f.proto = RftProto::FiatV2;
    else return false;

    if (f.bits == 0) {
        switch (f.proto) {
        case RftProto::HondaV1: f.bits = 68; break;
        case RftProto::ChryslerV0: f.bits = 80; break;
        case RftProto::KiaV1: f.bits = 57; break;
        case RftProto::FiatV2: f.bits = 112; break;
        default: break;
        }
    }
    if (f.te == 0) {
        switch (f.proto) {
        case RftProto::HondaV1: f.te = 1000; break;
        case RftProto::ChryslerV0: f.te = 300; break;
        case RftProto::KiaV1: f.te = 800; break;
        case RftProto::FiatV2: f.te = 210; break;
        default: break;
        }
    }
    frame = f;
    return true;
}
