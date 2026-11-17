// Protocol encoders for the automotive set. These mirror the ProtoPirate
// encoder framing (preamble, per-protocol gaps, counter/button/CRC fields) so
// a re-encoded frame looks like a real fob press rather than a raw replay.
#include "rft.h"

#include <string.h>

namespace {

inline void u64_to_bytes_be2(uint64_t v, uint8_t *b) {
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)((v >> ((7 - i) * 8)) & 0xFF);
}
inline bool bit_read2(uint64_t data, uint16_t index) { return ((data >> index) & 1ULL) != 0; }
inline uint8_t reverse6_2(uint32_t value) {
    uint8_t out = 0;
    uint8_t bits = 6;
    while (bits--) {
        out = (uint8_t)((out << 1U) | (value & 1U));
        value >>= 1U;
    }
    return out;
}

struct PulseBuilder {
    std::vector<int> pulses;

    void emit(bool level, uint32_t us) {
        if (us == 0) return;
        pulses.push_back(level ? (int)us : -(int)us);
    }

    void emitMerge(bool level, uint32_t us) {
        if (us == 0) return;
        if (!pulses.empty()) {
            const bool lastLevel = pulses.back() > 0;
            if (lastLevel == level) {
                pulses.back() = level ? (pulses.back() + (int)us) : (pulses.back() - (int)us);
                return;
            }
        }
        emit(level, us);
    }
};

// ---------------------------------------------------------------------------
// Honda V1
// ---------------------------------------------------------------------------

constexpr uint32_t HONDA_TE = 1000;
constexpr uint32_t HONDA_GAP = 5000;

uint32_t hondaButtonCode(uint8_t btn) {
    switch (btn) {
    case 0: return 0x00080808; // Unlock
    case 8: return 0x00088888; // Lock
    case 9: return 0x00099190; // Trunk
    case 10: return 0x000FA7A0; // Panic
    default: return 0x00088888;
    }
}

uint64_t hondaBuildKey(uint32_t serial, uint8_t btn, uint16_t cnt) {
    const uint32_t table = hondaButtonCode(btn);
    const uint32_t low = ((table & 0xFFFFU) << 16U) | cnt;
    const uint32_t high = ((serial & 0x0FFFFFFFU) << 4U) | (table >> 16U);
    return ((uint64_t)high << 32U) | low;
}

uint8_t hondaCrcFold(uint16_t v) {
    const uint8_t lo = (uint8_t)(v & 0x0FU);
    const uint16_t hi = (uint16_t)(v >> 4U);
    int32_t s = (hi & 1U) ? (int32_t)lo : -(int32_t)lo;
    uint8_t out = (uint8_t)((s - (int32_t)hi) & 7);
    out |= (uint8_t)(((v >> 3U) & 1U) << 3U);
    if (((v >> 1U) & 1U) && (((v >> 4U) ^ (v >> 5U)) & 1U)) out ^= 0x04U;
    return (uint8_t)(out & 0x0FU);
}

uint8_t hondaChecksumBase(uint64_t data) {
    const uint8_t a = hondaCrcFold((uint16_t)(data & 0xFFFFU));
    const uint8_t b = hondaCrcFold((uint8_t)((data >> 40U) & 0xFFU));
    return (uint8_t)((a ^ b ^ 1U) & 0x0FU);
}

uint8_t hondaChecksumAlternate(uint8_t checksum) {
    uint8_t mask = 0x09U;
    if ((checksum & 1U) == 0U) mask = (checksum & 2U) ? 0x0BU : 0x0FU;
    return (uint8_t)((checksum ^ mask) & 0x0FU);
}

void hondaChecksumWireOrder(uint64_t data, uint8_t *first, uint8_t *second) {
    const uint8_t checksum = hondaChecksumBase(data);
    const uint8_t other = hondaChecksumAlternate(checksum);
    if ((checksum & 0x08U) != 0U) {
        *first = other;
        *second = checksum;
    } else {
        *first = checksum;
        *second = other;
    }
}

void hondaAppendFrame(PulseBuilder &pb, const uint8_t frame[9]) {
    PulseBuilder g;
    for (uint32_t bitIndex = 0; bitIndex < 80; bitIndex++) {
        uint32_t bit;
        if (bitIndex >= 12) {
            const uint32_t dataIndex = (bitIndex - 12) >> 3;
            const uint8_t shift = (uint8_t)((11U - bitIndex) & 0x07U);
            bit = (frame[dataIndex] >> shift) & 0x01U;
        } else {
            bit = ((uint32_t)~bitIndex) & 0x01U;
        }
        g.emitMerge(bit != 0U, HONDA_TE);
        g.emitMerge(bit == 0U, HONDA_TE);
    }

    if (g.pulses.size() <= 2) return;
    for (size_t i = 2; i < g.pulses.size(); i++) pb.pulses.push_back(g.pulses[i]);

    const bool tailLevel = !(pb.pulses.back() > 0);
    pb.emit(tailLevel, HONDA_TE);
    if (!tailLevel) pb.emit(true, HONDA_TE);
    pb.emit(false, HONDA_GAP);
}

bool encodeHonda(RftFrame &frame, std::vector<int> &out) {
    uint8_t btn = (uint8_t)frame.btn;
    if (!(btn == 0 || btn == 8 || btn == 9 || btn == 10)) btn = 8;

    const uint64_t data = hondaBuildKey(frame.serial & 0x0FFFFFFFU, btn,
                                        (uint16_t)(frame.cnt & 0xFFFFU));

    uint8_t f[9] = {0};
    u64_to_bytes_be2(data, f);

    uint8_t first = 0, second = 0;
    hondaChecksumWireOrder(data, &first, &second);

    PulseBuilder pb;
    for (int i = 0; i < 90; i++) {
        pb.emit(true, HONDA_TE);
        pb.emit(false, HONDA_TE);
    }
    // Replace the last preamble low with the frame gap.
    if (!pb.pulses.empty()) pb.pulses.back() = -(int)HONDA_GAP;

    const uint8_t crcOrder[2] = {first, second};
    for (uint8_t crc : crcOrder) {
        f[8] = (uint8_t)(crc << 4U);
        for (int r = 0; r < 2; r++) hondaAppendFrame(pb, f);
    }

    if (pb.pulses.empty()) return false;

    frame.proto = RftProto::HondaV1;
    frame.key = data;
    frame.serial &= 0x0FFFFFFFU;
    frame.btn = btn;
    frame.cnt &= 0xFFFFU;
    frame.bits = 68;
    frame.te = HONDA_TE;
    frame.crc = second & 0x0F;
    frame.crcOk = true;
    out.swap(pb.pulses);
    return true;
}

// ---------------------------------------------------------------------------
// Chrysler V0
// ---------------------------------------------------------------------------

constexpr uint32_t CHR_TE_SHORT2 = 0x12C;
constexpr uint32_t CHR_TE_LONG_A2 = 0xD48;
constexpr uint32_t CHR_TE_LONG_B2 = 0xE74;
constexpr uint32_t CHR_TE_GAP2 = 0x3CF0;

void chryslerTransform(const uint8_t in[9], uint8_t out[9], uint32_t key, uint8_t button) {
    static const uint8_t table[16] = {0x0F, 0x02, 0x40, 0x0C, 0x30, 0x0E, 0x70, 0x08,
                                      0x10, 0x0A, 0x50, 0xF4, 0x2F, 0xF6, 0x6F, 0xF0};
    uint8_t mask = table[key & 0x0FU];
    if (button == 1U) mask ^= (key & 1U) ? 0xF0U : 0x0FU;
    for (size_t i = 0; i < 9; i++) out[i] = in[i] ^ mask;
}

void chryslerBuildPayload(
    const uint8_t plain[9],
    uint8_t counter,
    uint8_t button,
    uint8_t headerLow2,
    uint8_t out[10]) {
    uint8_t transformed[9];
    chryslerTransform(plain, transformed, counter, button);
    out[0] = (uint8_t)((reverse6_2(counter) << 2U) | (headerLow2 & 0x03U));
    memcpy(&out[1], transformed, 9);
}

uint8_t chryslerPayloadBit(const uint8_t payload[10], uint8_t index) {
    return (payload[index >> 3U] >> (7U - (index & 7U))) & 1U;
}

void chryslerAppendPayload(PulseBuilder &pb, const uint8_t payload[10]) {
    for (uint8_t bit = 0; bit < 80; bit++) {
        const bool value = chryslerPayloadBit(payload, bit) != 0;
        pb.emit(true, value ? 0x258U : CHR_TE_SHORT2);
        pb.emit(false, value ? CHR_TE_LONG_A2 : CHR_TE_LONG_B2);
    }
    pb.emit(true, CHR_TE_SHORT2);
    pb.emit(false, CHR_TE_GAP2);
}

bool encodeChrysler(RftFrame &frame, std::vector<int> &out) {
    uint8_t plainA[9];
    uint8_t plainB[9];

    if (!frame.plainAPresent && !frame.plainBPresent) {
        // Rebuild the plain block from the encoded key, exactly like the
        // ProtoPirate encoder does when a .key carries only Key/Key_2.
        uint8_t key[8];
        u64_to_bytes_be2(frame.key, key);
        uint8_t encoded[9];
        encoded[0] = key[1];
        encoded[1] = key[2];
        encoded[2] = key[3];
        encoded[3] = key[4];
        encoded[4] = key[5];
        encoded[5] = key[6];
        encoded[6] = key[7];
        encoded[7] = (uint8_t)(frame.key2 >> 8U);
        encoded[8] = (uint8_t)(frame.key2 & 0xFFU);
        chryslerTransform(encoded, plainA, frame.seed, (uint8_t)frame.btn);
        memcpy(plainB, plainA, 9);
    } else if (frame.plainAPresent && !frame.plainBPresent) {
        memcpy(plainA, frame.plainA, 9);
        memcpy(plainB, frame.plainA, 9);
    } else if (!frame.plainAPresent && frame.plainBPresent) {
        memcpy(plainB, frame.plainB, 9);
        memcpy(plainA, frame.plainB, 9);
    } else {
        memcpy(plainA, frame.plainA, 9);
        memcpy(plainB, frame.plainB, 9);
    }

    uint8_t button = (uint8_t)frame.btn;
    if (button != 1U && button != 2U) button = 1U;

    uint8_t counterA = (uint8_t)(frame.cnt & 0x3FU);
    if (counterA & 1U) counterA = (uint8_t)((counterA - 1U) & 0x3FU);
    const uint8_t counterB = (counterA == 0U) ? 0x3FU : (uint8_t)(counterA - 1U);

    uint8_t payloadA[10];
    uint8_t payloadB[10];
    chryslerBuildPayload(plainA, counterA, button, frame.header, payloadA);
    chryslerBuildPayload(plainB, counterB, button, frame.header, payloadB);

    PulseBuilder pb;
    for (int i = 0; i < 24; i++) {
        pb.emit(true, CHR_TE_SHORT2);
        pb.emit(false, CHR_TE_LONG_B2);
    }
    pb.emit(true, CHR_TE_SHORT2);
    pb.emit(false, CHR_TE_GAP2);
    chryslerAppendPayload(pb, payloadA);
    for (int i = 0; i < 24; i++) {
        pb.emit(true, CHR_TE_SHORT2);
        pb.emit(false, CHR_TE_LONG_B2);
    }
    pb.emit(true, CHR_TE_SHORT2);
    pb.emit(false, CHR_TE_GAP2);
    chryslerAppendPayload(pb, payloadB);

    if (pb.pulses.empty()) return false;

    frame.proto = RftProto::ChryslerV0;
    frame.key = 0;
    for (int i = 0; i < 8; i++) frame.key = (frame.key << 8) | payloadA[i];
    frame.key2 = (uint16_t)((payloadA[8] << 8) | payloadA[9]);
    frame.btn = button;
    frame.seed = counterA;
    frame.cnt = (frame.cnt & ~0x3FU) | counterA;
    frame.bits = 80;
    frame.te = CHR_TE_SHORT2;
    memcpy(frame.plainA, plainA, 9);
    memcpy(frame.plainB, plainB, 9);
    frame.plainAPresent = true;
    frame.plainBPresent = true;
    out.swap(pb.pulses);
    return true;
}

// ---------------------------------------------------------------------------
// Kia V1
// ---------------------------------------------------------------------------

constexpr uint32_t KIA_TE_SHORT2 = 800;
constexpr uint32_t KIA_TE_LONG2 = 1600;
constexpr uint32_t KIA_GAP2 = 25000;

uint8_t kiaCrc4(const uint8_t *bytes, int count, uint8_t offset) {
    uint8_t crc = 0;
    for (int i = 0; i < count; i++) {
        const uint8_t b = bytes[i];
        crc ^= ((b & 0x0F) ^ (b >> 4));
    }
    return (uint8_t)((crc + offset) & 0x0F);
}

void kiaEmitManchester(PulseBuilder &pb, bool bit) {
    pb.emitMerge(bit, KIA_TE_SHORT2);
    pb.emitMerge(!bit, KIA_TE_SHORT2);
}

bool encodeKia(RftFrame &frame, std::vector<int> &out) {
    const uint32_t serial = frame.serial;
    const uint8_t btn = (uint8_t)frame.btn;
    const uint16_t cnt = (uint16_t)(frame.cnt & 0xFFFU);

    uint8_t charData[7];
    charData[0] = (uint8_t)(serial >> 24);
    charData[1] = (uint8_t)(serial >> 16);
    charData[2] = (uint8_t)(serial >> 8);
    charData[3] = (uint8_t)serial;
    charData[4] = btn;
    charData[5] = (uint8_t)(cnt & 0xFF);
    charData[6] = (uint8_t)((cnt >> 8) & 0xF);
    const uint8_t crc = kiaCrc4(charData, 7, 1);

    // 56-bit payload: serial(32) + button(8) + counter(12) + crc(4).
    const uint64_t data = ((uint64_t)serial << 24) | ((uint64_t)btn << 16) |
                          ((uint64_t)(cnt & 0xFF) << 8) | ((uint64_t)((cnt >> 8) & 0xF) << 4) |
                          crc;

    PulseBuilder pb;
    for (uint8_t burst = 0; burst < 3; burst++) {
        if (burst > 0) pb.emit(false, KIA_GAP2);
        for (int i = 0; i < 90; i++) {
            pb.emit(false, KIA_TE_LONG2);
            pb.emit(true, KIA_TE_LONG2);
        }
        // Separator keeps the last header high from merging into the start bit.
        pb.emit(false, KIA_TE_SHORT2);
        // Start bit (always 1) + 56 payload bits, MSB first.
        kiaEmitManchester(pb, true);
        for (int i = 55; i >= 0; i--) {
            kiaEmitManchester(pb, bit_read2(data, (uint16_t)i));
        }
    }

    if (pb.pulses.empty()) return false;

    frame.proto = RftProto::KiaV1;
    frame.key = (1ULL << 56) | data;
    frame.cnt = cnt;
    frame.bits = 57;
    frame.te = KIA_TE_SHORT2;
    frame.crc = crc;
    frame.crcOk = true;
    out.swap(pb.pulses);
    return true;
}

} // namespace

bool rft_encode(RftFrame &frame, std::vector<int> &out) {
    out.clear();
    switch (frame.proto) {
    case RftProto::HondaV1: return encodeHonda(frame, out);
    case RftProto::ChryslerV0: return encodeChrysler(frame, out);
    case RftProto::KiaV1: return encodeKia(frame, out);
    default: return false;
    }
}
