#include "rf_weather_decoders.h"

#include <string.h>

namespace {

struct BitFrame {
    uint8_t bytes[10] = {0};
    int bits = 0;

    void add(bool bit) {
        if (bits >= 80) return;
        if (bit) bytes[bits >> 3] |= (uint8_t)(1U << (7 - (bits & 7)));
        bits++;
    }
    void clear() {
        memset(bytes, 0, sizeof(bytes));
        bits = 0;
    }
};

uint64_t frame_u64(const BitFrame &f) {
    uint64_t v = 0;
    const int nbytes = (f.bits + 7) / 8;
    for (int i = 0; i < nbytes; i++) v = (v << 8) | f.bytes[i];
    const int pad = nbytes * 8 - f.bits;
    return pad > 0 ? (v >> pad) : v;
}

uint32_t slice(uint64_t frame, int width, int a, int b) {
    const int shift = width - b;
    const uint32_t mask = (b - a >= 32) ? 0xFFFFFFFFu : ((1u << (b - a)) - 1u);
    return (uint32_t)((frame >> shift) & mask);
}

int32_t signed12(uint32_t n) { return n >= 2048 ? (int32_t)n - 4096 : (int32_t)n; }

float f_to_c(float f) { return (f - 32.0f) * 5.0f / 9.0f; }

bool even_parity(uint8_t v) { return (__builtin_popcount(v) & 1) == 0; }

// Non-reflected LFSR digest (AcuRite 606TX / rtl_433 lfsr_digest8).
uint8_t lfsr8(const uint8_t *msg, int n, uint8_t gen, uint8_t key) {
    uint8_t sum = 0;
    for (int k = 0; k < n; k++) {
        const uint8_t data = msg[k];
        for (int i = 7; i >= 0; i--) {
            if ((data >> i) & 1) sum ^= key;
            key = (key & 1) ? (uint8_t)((key >> 1) ^ gen) : (uint8_t)(key >> 1);
        }
    }
    return sum;
}

// Reflected LFSR digest (LaCrosse TX141TH / rtl_433 lfsr_digest8_reflect).
uint8_t lfsr8_reflect(const uint8_t *msg, int n, uint8_t gen, uint8_t key) {
    uint8_t sum = 0;
    for (int byte = n - 1; byte >= 0; byte--) {
        const uint8_t data = msg[byte];
        for (int i = 0; i < 8; i++) {
            if ((data >> i) & 1) sum ^= key;
            key = (key & 0x80) ? (uint8_t)((key << 1) ^ gen) : (uint8_t)(key << 1);
        }
    }
    return sum;
}

// --- PPM extraction -------------------------------------------------------
// A short LOW gap is a 0, a long LOW gap is a 1. Gaps >= reset end the frame.
std::vector<BitFrame> ppm_frames(
    const std::vector<int> &durations,
    int shortMin,
    int shortMax,
    int longMin,
    int longMax,
    int resetGap,
    int bitCount) {
    std::vector<BitFrame> out;
    BitFrame cur;
    for (int d : durations) {
        if (d >= 0) continue;
        const int gap = -d;
        if (gap >= resetGap) {
            if (cur.bits == bitCount) out.push_back(cur);
            cur.clear();
            continue;
        }
        if (gap >= shortMin && gap < shortMax) {
            cur.add(false);
        } else if (gap >= longMin && gap < longMax) {
            cur.add(true);
        } else {
            if (cur.bits == bitCount) out.push_back(cur);
            cur.clear();
        }
        if (cur.bits > bitCount) cur.clear();
    }
    if (cur.bits == bitCount) out.push_back(cur);
    return out;
}

// --- AcuRite TXR PWM ------------------------------------------------------
// 592TXR/Tower and friends: 220 us high + 392 us low = 0, 408 us high +
// 204 us low = 1, ~620 us high + ~596 us low = sync. Thresholds are widened
// for real-world captures, which run shorter on the high side.
std::vector<BitFrame> acurite_pwm_frames(const std::vector<int> &durations) {
    std::vector<BitFrame> out;
    BitFrame cur;
    const size_t n = durations.size();
    size_t i = 0;
    while (i + 1 < n) {
        if (durations[i] <= 0 || durations[i + 1] >= 0) {
            i++;
            continue;
        }
        const int hi = durations[i];
        const int lo = -durations[i + 1];

        // Sync pulse or packet gap: flush a complete frame.
        if (lo > 560 || hi > 560) {
            if (cur.bits == 56 || cur.bits == 64 || cur.bits == 72 || cur.bits == 80) {
                out.push_back(cur);
            }
            cur.clear();
            i++;
            continue;
        }

        bool bit;
        bool ok = true;
        if (hi >= 100 && hi < 300 && lo >= 250 && lo < 560) {
            bit = false;
        } else if (hi >= 300 && hi < 560 && lo >= 80 && lo < 340) {
            bit = true;
        } else {
            ok = false;
            bit = false;
        }

        if (!ok) {
            if (cur.bits == 56 || cur.bits == 64 || cur.bits == 72 || cur.bits == 80) {
                out.push_back(cur);
            }
            cur.clear();
            i++;
            continue;
        }

        cur.add(bit);
        if (cur.bits > 80) cur.clear();
        i += 2;
    }
    if (cur.bits == 56 || cur.bits == 64 || cur.bits == 72 || cur.bits == 80) {
        out.push_back(cur);
    }
    return out;
}

// --- LaCrosse TX141TH PWM -------------------------------------------------
// Bit 1 = 417 us high + 208 us low, bit 0 = 208 us high + 417 us low. The
// 833 us preamble/postamble pulses reset the frame.
std::vector<BitFrame> lacrosse_pwm_frames(const std::vector<int> &durations) {
    std::vector<BitFrame> out;
    BitFrame cur;
    const size_t n = durations.size();
    size_t i = 0;
    while (i + 1 < n) {
        if (durations[i] <= 0 || durations[i + 1] >= 0) {
            i++;
            continue;
        }
        const int hi = durations[i];
        const int lo = -durations[i + 1];

        if (hi > 600 || lo > 600) {
            if (cur.bits == 40) out.push_back(cur);
            cur.clear();
            i++;
            continue;
        }

        bool bit;
        bool ok = true;
        if (hi >= 150 && hi < 300 && lo >= 300 && lo < 560) bit = false;
        else if (hi >= 300 && hi < 560 && lo >= 100 && lo < 320) bit = true;
        else {
            ok = false;
            bit = false;
        }

        if (!ok) {
            if (cur.bits == 40) out.push_back(cur);
            cur.clear();
            i++;
            continue;
        }

        cur.add(bit);
        if (cur.bits == 40) {
            out.push_back(cur);
            cur.clear();
        }
        i += 2;
    }
    if (cur.bits == 40) out.push_back(cur);
    return out;
}

// --- Protocol decoders ----------------------------------------------------

bool decode_acurite606(uint64_t frame, WeatherReading &out) {
    uint8_t b[4];
    for (int i = 0; i < 4; i++) b[i] = (uint8_t)((frame >> (8 * (3 - i))) & 0xFF);
    if (b[0] == 0 && b[1] == 0 && b[2] == 0 && b[3] == 0) return false;
    if (lfsr8(b, 3, 0x98, 0xF1) != b[3]) return false;

    const int raw = ((b[1] & 0x0F) << 8) | b[2];
    const int signed_raw = (raw & 0x800) ? raw - 0x1000 : raw;
    const float temp = signed_raw * 0.1f;
    if (temp < -50.0f || temp > 80.0f) return false;

    out.model = "AcuRite-606TX";
    out.id = b[0];
    out.channel = ((b[1] & 0x30) >> 4) + 1;
    out.battery = (b[1] & 0x80) != 0;
    out.tempC = temp;
    out.hasTemp = true;
    out.hasHumidity = false;
    memcpy(out.raw, b, 4);
    out.rawBits = 32;
    return true;
}

int acurite_channel(uint8_t b0) {
    switch ((b0 & 0xC0) >> 6) {
    case 0: return 1; // C
    case 2: return 2; // B
    case 3: return 3; // A
    default: return 0;
    }
}

bool acurite_txr_mic(const BitFrame &f, int nbytes) {
    if (f.bits != nbytes * 8) return false;
    uint8_t sum = 0;
    for (int i = 0; i < nbytes - 1; i++) sum = (uint8_t)(sum + f.bytes[i]);
    if (sum != f.bytes[nbytes - 1]) return false;
    for (int i = 2; i < nbytes - 1; i++) {
        if (!even_parity(f.bytes[i])) return false;
    }
    return true;
}

bool decode_acurite_txr(const BitFrame &f, WeatherReading &out) {
    const int n = f.bits / 8;
    const uint8_t *b = f.bytes;

    if (n == 7 && (b[2] & 0x3F) == 0x04) {
        if (!acurite_txr_mic(f, 7)) return false;
        const int ch = acurite_channel(b[0]);
        if (ch == 0) return false;
        const int hum = b[3] & 0x7F;
        if (hum > 100 && hum != 127) return false;
        const int raw = ((b[4] & 0x7F) << 7) | (b[5] & 0x7F);
        const float temp = (raw - 1000) * 0.1f;
        if (temp < -40.0f || temp > 70.0f) return false;

        out.model = "AcuRite-Tower";
        out.id = ((b[0] & 0x3F) << 8) | b[1];
        out.channel = ch;
        out.battery = (b[2] & 0x40) != 0;
        out.tempC = temp;
        out.hasTemp = true;
        out.hasHumidity = (hum > 0 && hum <= 100);
        out.humidity = hum;
        memcpy(out.raw, b, 7);
        out.rawBits = 56;
        return true;
    }

    if (n == 8) {
        const uint8_t type = b[2] & 0x3F;
        if (type == 0x20) { // 3n1: wind + temp + humidity
            if (!acurite_txr_mic(f, 8)) return false;
            const int ch = acurite_channel(b[0]);
            if (ch == 0) return false;
            const int hum = b[3] & 0x7F;
            if (hum > 100) return false;
            const int raw = ((b[4] & 0x1F) << 7) | (b[5] & 0x7F);
            const float tempF = (raw - 1480) * 0.1f;
            if (tempF < -40.0f || tempF > 158.0f) return false;
            const float windMph = (float)(b[6] & 0x7F);

            out.model = "AcuRite-3n1";
            out.id = ((b[0] & 0x3F) << 8) | b[1];
            out.channel = ch;
            out.battery = (b[2] & 0x40) != 0;
            out.tempC = f_to_c(tempF);
            out.hasTemp = true;
            out.hasHumidity = true;
            out.humidity = hum;
            out.hasWind = true;
            out.windKph = windMph * 1.609344f;
            memcpy(out.raw, b, 8);
            out.rawBits = 64;
            return true;
        }
        if (type == 0x38) { // 5n1: wind + temp + humidity
            if (!acurite_txr_mic(f, 8)) return false;
            const int ch = acurite_channel(b[0]);
            if (ch == 0) return false;
            const int hum = b[6] & 0x7F;
            if (hum > 100) return false;
            const int raw = ((b[4] & 0x0F) << 7) | (b[5] & 0x7F);
            const float tempF = (raw - 400) * 0.1f;
            if (tempF < -40.0f || tempF > 158.0f) return false;
            const int windRaw = ((b[3] & 0x1F) << 3) | ((b[4] & 0x70) >> 4);

            out.model = "AcuRite-5n1";
            out.id = ((b[0] & 0x0F) << 8) | b[1];
            out.channel = ch;
            out.battery = (b[2] & 0x40) != 0;
            out.tempC = f_to_c(tempF);
            out.hasTemp = true;
            out.hasHumidity = true;
            out.humidity = hum;
            out.hasWind = windRaw > 0;
            out.windKph = windRaw > 0 ? (windRaw * 0.8278f + 1.0f) : 0.0f;
            memcpy(out.raw, b, 8);
            out.rawBits = 64;
            return true;
        }
        if (type == 0x31) { // 5n1: wind + rain
            if (!acurite_txr_mic(f, 8)) return false;
            const int ch = acurite_channel(b[0]);
            if (ch == 0) return false;
            const int windRaw = ((b[3] & 0x1F) << 3) | ((b[4] & 0x70) >> 4);
            const int rainCounter = ((b[5] & 0x7F) << 7) | (b[6] & 0x7F);

            out.model = "AcuRite-5n1";
            out.id = ((b[0] & 0x0F) << 8) | b[1];
            out.channel = ch;
            out.battery = (b[2] & 0x40) != 0;
            out.hasTemp = false;
            out.hasHumidity = false;
            out.hasWind = windRaw > 0;
            out.windKph = windRaw > 0 ? (windRaw * 0.8278f + 1.0f) : 0.0f;
            out.hasRain = true;
            out.rainMm = rainCounter * 0.254f;
            memcpy(out.raw, b, 8);
            out.rawBits = 64;
            return true;
        }
    }

    if (n == 9 && (b[2] & 0x3F) == 0x2F) { // 6045M lightning
        if (!acurite_txr_mic(f, 9)) return false;
        const int ch = acurite_channel(b[0]);
        if (ch == 0) return false;
        const int hum = b[3] & 0x7F;
        if (hum > 100) return false;
        const int raw = ((b[4] & 0x1F) << 7) | (b[5] & 0x7F);
        const float tempF = (raw - 1480) * 0.1f;
        if (tempF < -40.0f || tempF > 158.0f) return false;

        out.model = "AcuRite-6045M";
        out.id = ((b[0] & 0x3F) << 8) | b[1];
        out.channel = ch;
        out.battery = (b[2] & 0x40) != 0;
        out.tempC = f_to_c(tempF);
        out.hasTemp = true;
        out.hasHumidity = true;
        out.humidity = hum;
        memcpy(out.raw, b, 9);
        out.rawBits = 72;
        return true;
    }

    if (n == 10) {
        const uint8_t type = b[2] & 0x3F;
        if (type == 0x25 || type == 0x26 || type == 0x27) {
            if (!acurite_txr_mic(f, 10)) return false;
            const int ch = acurite_channel(b[0]);
            if (ch == 0) return false;
            const int windMph = ((b[3] & 0x7F) << 1) | ((b[4] & 0x40) >> 6);

            out.model = "AcuRite-Atlas";
            out.id = ((b[0] & 0x03) << 8) | b[1];
            out.channel = ch;
            out.battery = (b[2] & 0x40) != 0;
            out.hasWind = windMph > 0;
            out.windKph = windMph * 1.609344f;
            out.hasTemp = false;
            out.hasHumidity = false;

            if (type == 0x25) {
                const int raw = ((b[4] & 0x0F) << 7) | (b[5] & 0x7F);
                const float tempF = (raw - 400) * 0.1f;
                const int hum = b[6] & 0x7F;
                if (tempF < -40.0f || tempF > 158.0f || hum > 100) return false;
                out.tempC = f_to_c(tempF);
                out.hasTemp = true;
                out.hasHumidity = true;
                out.humidity = hum;
            } else if (type == 0x26) {
                const int rainCounter = ((b[5] & 0x03) << 7) | (b[6] & 0x7F);
                out.hasRain = true;
                out.rainMm = rainCounter * 0.254f;
            }
            memcpy(out.raw, b, 10);
            out.rawBits = 80;
            return true;
        }
    }

    return false;
}

bool decode_nexus(uint64_t frame, WeatherReading &out) {
    if (slice(frame, 36, 24, 28) != 0xF) return false;
    const uint32_t flags = slice(frame, 36, 8, 12);
    const int channel = (flags & 3) + 1;
    const float temp = signed12(slice(frame, 36, 12, 24)) / 10.0f;
    const int hum = (int)slice(frame, 36, 28, 36);
    if (channel > 3 || temp < -60.0f || temp > 80.0f) return false;

    // Several temperature-only Nexus clones use 0xFE/0xFF/0xCC as "no
    // humidity" instead of 0.
    bool hasHum = (hum > 0 && hum <= 100);
    if (!hasHum) {
        if (hum != 0 && hum != 0x7F && hum != 0xCC && hum != 0xFE && hum != 0xFF) return false;
    }

    out.model = hasHum ? "Nexus-TH" : "Nexus-T";
    out.id = (int)slice(frame, 36, 0, 8);
    out.channel = channel;
    out.battery = (flags & 8) != 0;
    out.tempC = temp;
    out.hasTemp = true;
    out.hasHumidity = hasHum;
    out.humidity = hasHum ? hum : 0;
    out.rawBits = 36;
    return true;
}

bool decode_prologue(uint64_t frame, WeatherReading &out) {
    const uint32_t type = slice(frame, 36, 0, 4);
    if (type != 9 && type != 5) return false;
    const uint32_t id = slice(frame, 36, 4, 12);
    const uint32_t flags = slice(frame, 36, 12, 16);
    const float temp = signed12(slice(frame, 36, 16, 28)) / 10.0f;
    const int hum = (int)slice(frame, 36, 28, 36);
    if (temp < -60.0f || temp > 80.0f) return false;
    const bool hasHum = (hum != 204 && hum <= 100);
    if (!hasHum && hum != 204 && hum > 100) return false;

    out.model = "Prologue-TH";
    out.id = (int)id;
    out.channel = (flags & 3) + 1;
    out.battery = (flags & 8) != 0;
    out.tempC = temp;
    out.hasTemp = true;
    out.hasHumidity = hasHum;
    out.humidity = hasHum ? hum : 0;
    out.rawBits = 36;
    return true;
}

bool decode_lacrosse(const BitFrame &f, WeatherReading &out) {
    if (f.bits != 40) return false;
    const uint8_t *b = f.bytes;
    if (lfsr8_reflect(b, 4, 0x31, 0xF4) != b[4]) return false;
    if (b[0] == 0) return false;
    const int hum = b[3];
    if (hum == 0 || hum > 100) return false;
    const int raw = ((b[1] & 0x0F) << 8) | b[2];
    const float temp = (raw - 500) * 0.1f;
    if (temp < -40.0f || temp > 140.0f) return false;

    out.model = "LaCrosse-TX141TH";
    out.id = b[0];
    out.channel = ((b[1] & 0x30) >> 4) + 1;
    out.battery = (b[1] & 0x80) == 0; // battery-low bit
    out.tempC = temp;
    out.hasTemp = true;
    out.hasHumidity = true;
    out.humidity = hum;
    memcpy(out.raw, b, 5);
    out.rawBits = 40;
    return true;
}

int bump_repeat(
    std::vector<WeatherRepeat> &repeats,
    int profile,
    uint64_t frame,
    uint32_t now,
    uint32_t timeoutMs) {
    for (size_t i = 0; i < repeats.size();) {
        if (now - repeats[i].lastMs > timeoutMs) repeats.erase(repeats.begin() + i);
        else i++;
    }
    for (auto &r : repeats) {
        if (r.profile == profile && r.frame == frame) {
            r.lastMs = now;
            return ++r.count;
        }
    }
    if (repeats.size() >= 24) repeats.erase(repeats.begin());
    repeats.push_back({profile, frame, 1, now});
    return 1;
}

} // namespace

bool weather_decode(
    const std::vector<int> &durations,
    std::vector<WeatherRepeat> &repeats,
    WeatherReading &out,
    int minRepeats) {
    // Strong, checksum-validated protocols first.
    for (const auto &f : acurite_pwm_frames(durations)) {
        if (decode_acurite_txr(f, out)) return true;
    }
    for (const auto &f : lacrosse_pwm_frames(durations)) {
        if (decode_lacrosse(f, out)) return true;
    }
    for (const auto &f : ppm_frames(durations, 1600, 2800, 3150, 4700, 7000, 32)) {
        if (decode_acurite606(frame_u64(f), out)) return true;
    }

    // Repeated-frame protocols (no checksum: require identical repeats live).
    const uint32_t now = millis();
    for (const auto &f : ppm_frames(durations, 450, 1450, 1450, 2900, 3200, 36)) {
        const uint64_t frame = frame_u64(f);
        const int count = bump_repeat(repeats, 0, frame, now, 15000);
        if (count >= minRepeats && decode_nexus(frame, out)) return true;
    }
    for (const auto &f : ppm_frames(durations, 1300, 2850, 2850, 6200, 7000, 36)) {
        const uint64_t frame = frame_u64(f);
        const int count = bump_repeat(repeats, 1, frame, now, 15000);
        if (count >= minRepeats && decode_prologue(frame, out)) return true;
    }

    return false;
}

String weather_diagnose(const std::vector<int> &durations) {
    const size_t txr = acurite_pwm_frames(durations).size();
    const size_t lac = lacrosse_pwm_frames(durations).size();
    const size_t acu = ppm_frames(durations, 1600, 2800, 3150, 4700, 7000, 32).size();
    const size_t nex = ppm_frames(durations, 450, 1450, 1450, 2900, 3200, 36).size();
    const size_t pro = ppm_frames(durations, 1300, 2850, 2850, 6200, 7000, 36).size();

    String s;
    s += "pulses: ";
    s += String((int)durations.size());
    s += "\nTXR:";
    s += String((int)txr);
    s += " LaC:";
    s += String((int)lac);
    s += " 606:";
    s += String((int)acu);
    s += "\nNex:";
    s += String((int)nex);
    s += " Pro:";
    s += String((int)pro);
    return s;
}

std::vector<int> weather_parse_sub(const String &text) {
    std::vector<int> out;
    int start = 0;
    while (start < (int)text.length()) {
        int end = text.indexOf('\n', start);
        if (end < 0) end = text.length();
        String line = text.substring(start, end);
        start = end + 1;
        if (line.endsWith("\r")) line.remove(line.length() - 1);
        if (!line.startsWith("RAW_Data:")) continue;

        int i = 9;
        while (i < (int)line.length()) {
            while (i < (int)line.length() &&
                   (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) {
                i++;
            }
            if (i >= (int)line.length()) break;
            int sign = 1;
            if (line[i] == '-') {
                sign = -1;
                i++;
            } else if (line[i] == '+') {
                i++;
            }
            if (i >= (int)line.length() || line[i] < '0' || line[i] > '9') {
                while (i < (int)line.length() && line[i] != ' ') i++;
                continue;
            }
            long v = 0;
            while (i < (int)line.length() && line[i] >= '0' && line[i] <= '9') {
                v = v * 10 + (line[i] - '0');
                i++;
            }
            out.push_back((int)(sign * v));
        }
    }
    return out;
}

const char *weather_protocol_name(uint8_t index) {
    switch (index) {
    case 0: return "AcuRite 606TX";
    case 1: return "AcuRite TXR";
    case 2: return "Nexus";
    case 3: return "Prologue";
    case 4: return "LaCrosse TX141";
    default: return "Unknown";
    }
}
