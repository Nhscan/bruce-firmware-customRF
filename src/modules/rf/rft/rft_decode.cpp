// Native decoders for the four automotive protocols. These are ports of the
// ProtoPirate/Flipper state machines (same constants, same framing, same
// checks) re-written against a plain signed-duration buffer so they do not
// need Flipper's SubGhz HAL.
#include "rft.h"

#include <string.h>

namespace {

inline uint32_t rft_absd(int v) { return (uint32_t)(v < 0 ? -v : v); }

inline bool near(uint32_t d, uint32_t t, uint32_t delta) {
    return (d >= t) ? ((d - t) <= delta) : ((t - d) <= delta);
}

inline uint64_t bytes_to_u64_be(const uint8_t *b) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v = (v << 8) | b[i];
    return v;
}

inline void u64_to_bytes_be(uint64_t v, uint8_t *b) {
    for (int i = 0; i < 8; i++) b[i] = (uint8_t)((v >> ((7 - i) * 8)) & 0xFF);
}

inline bool bit_read(uint64_t data, uint16_t index) { return ((data >> index) & 1ULL) != 0; }

inline uint8_t reverse6(uint32_t value) {
    uint8_t out = 0;
    uint8_t bits = 6;
    while (bits--) {
        out = (uint8_t)((out << 1U) | (value & 1U));
        value >>= 1U;
    }
    return out;
}

// ---------------------------------------------------------------------------
// Honda V1 -- 68-bit PWM, ~1 ms / 2 ms, 0x0FFFFFFF serial, 16-bit counter.
// ---------------------------------------------------------------------------

constexpr uint32_t HONDA_BITS = 68;
constexpr uint32_t HONDA_TE_SHORT = 1000;
constexpr uint32_t HONDA_TE_LONG = 2000;
constexpr uint32_t HONDA_TE_DELTA = 400;
constexpr uint32_t HONDA_TE_SHORT_MIN = 600;
constexpr uint32_t HONDA_TE_END = 3500;
constexpr uint8_t HONDA_VALID_MAX = 0x4B;
constexpr uint32_t HONDA_SERIAL_MASK = 0x0FFFFFFF;

struct HondaDecoder {
    enum Step { Reset = 0, Preamble, Data } step = Reset;
    uint8_t preambleCount = 0;
    bool preambleHasLong = false;
    bool dataPending = false;
    bool lastLevel = false;
    uint8_t bits[12] = {0};
    uint8_t bitCount = 0;
    uint32_t pending = 0;
    bool pendingValid = false;
    uint8_t k2 = 0;
    uint32_t commits = 0;
    RftFrame frame;

    static bool buttonValid(uint8_t b) {
        if (b > 10) return false;
        return ((0x701U >> b) & 1U) != 0U;
    }

    void reset() {
        step = Reset;
        preambleCount = 0;
        preambleHasLong = false;
        dataPending = false;
        lastLevel = false;
        bitCount = 0;
        memset(bits, 0, sizeof(bits));
        pending = 0;
        pendingValid = false;
        k2 = 0;
    }

    void addBit(bool bit) {
        if (bitCount > HONDA_VALID_MAX) return;
        if (bit) bits[bitCount >> 3] |= (uint8_t)(1U << ((uint8_t)~bitCount & 0x07U));
        bitCount++;
    }

    bool commit() {
        if (bitCount < HONDA_BITS) return false;

        uint8_t aligned[sizeof(bits)];
        memcpy(aligned, bits, sizeof(aligned));

        uint8_t shiftCount = (uint8_t)(bitCount - HONDA_BITS);
        if (shiftCount < 1U) shiftCount = 1U;
        for (uint8_t s = 0; s < shiftCount; s++) {
            for (size_t i = 0; i < sizeof(aligned) - 1U; i++) {
                aligned[i] = (uint8_t)((aligned[i] << 1U) | (aligned[i + 1U] >> 7U));
            }
            aligned[sizeof(aligned) - 1U] <<= 1U;
        }

        const uint8_t button = (uint8_t)(aligned[4] >> 4U);
        if (!buttonValid(button)) return false;

        const uint64_t data = bytes_to_u64_be(aligned);
        k2 = (uint8_t)(aligned[8] >> 4U);

        if (commits == 0) {
            frame.proto = RftProto::HondaV1;
            frame.key = data;
            frame.serial = (uint32_t)((data >> 36U) & HONDA_SERIAL_MASK);
            frame.btn = (uint8_t)(((uint32_t)data >> 28U) & 0x0FU);
            frame.cnt = (uint32_t)data & 0xFFFFU;
            frame.bits = HONDA_BITS;
            frame.te = HONDA_TE_SHORT;
            frame.crc = k2 & 0x0F;
            frame.crcOk = true;
        }
        commits++;
        return true;
    }

    void stateReset() {
        step = Reset;
        preambleCount = 0;
        preambleHasLong = false;
        dataPending = false;
        lastLevel = false;
        bitCount = 0;
        memset(bits, 0, sizeof(bits));
    }

    void symbol(bool level, uint32_t duration) {
        const bool sh = near(duration, HONDA_TE_SHORT, HONDA_TE_DELTA);
        const bool lg = near(duration, HONDA_TE_LONG, HONDA_TE_DELTA);

        if (!sh && !lg) {
            if (!level && duration > HONDA_TE_END && step == Data) commit();
            stateReset();
            return;
        }

        if (step == Reset) {
            if (level) {
                step = Preamble;
                preambleCount = 1U;
                lastLevel = level;
            }
            return;
        }

        if (step == Preamble) {
            if (lg) {
                if (preambleCount < 0xFFU) preambleCount++;
                preambleHasLong = true;
                lastLevel = level;
                return;
            }
            if (sh) {
                if (preambleHasLong && preambleCount > 5U) {
                    step = Data;
                    bitCount = 0U;
                    memset(bits, 0, sizeof(bits));
                    dataPending = true;
                    lastLevel = level;
                    return;
                }
                if (preambleCount < 0xFFU) preambleCount++;
                lastLevel = level;
                return;
            }
            stateReset();
            return;
        }

        if (sh) {
            if (dataPending) {
                addBit(level);
                dataPending = false;
                lastLevel = level;
                return;
            }
            dataPending = true;
            lastLevel = level;
        } else {
            if (dataPending) {
                addBit(level);
            } else {
                addBit(lastLevel);
            }
            lastLevel = level;
        }
    }

    void feed(bool level, uint32_t duration) {
        if (duration < HONDA_TE_DELTA) {
            pending += duration;
            pendingValid = true;
            return;
        }

        if (pendingValid) {
            const uint32_t p = pending;
            if (level) {
                pending = p + duration;
                pendingValid = true;
                return;
            }
            if (p >= HONDA_TE_SHORT_MIN) symbol(true, p);
            pending = 0U;
            pendingValid = false;
        }

        if (level) {
            pending = duration;
            pendingValid = true;
            return;
        }

        symbol(false, duration);
    }
};

// ---------------------------------------------------------------------------
// Chrysler V0 -- 80-bit packet split into two 40-bit halves plus a 2-byte tail.
// ---------------------------------------------------------------------------

constexpr uint32_t CHR_TE_SHORT = 0x12C;
constexpr uint32_t CHR_TE_DELTA = 0x96;
constexpr uint32_t CHR_TE_LONG_A = 0xD48;
constexpr uint32_t CHR_TE_LONG_B = 0xE74;
constexpr uint32_t CHR_TE_LONG_DELTA = 0x190;
constexpr uint32_t CHR_TE_GAP = 0x1F40;
constexpr uint8_t CHR_DECODE_BIT_COUNT = 0x50;

struct ChryslerDecoder {
    enum Step { Reset = 0, Seek = 1, Data = 2 } step = Reset;

    uint64_t decodeData = 0;
    uint16_t decodeCount = 0;
    uint16_t packetBitCount = 0;
    uint32_t teLast = 0;
    uint16_t data2 = 0;

    uint32_t commits = 0;
    RftFrame frame;

    static bool isShort(uint32_t d) { return rft_absd((int)d - (int)CHR_TE_SHORT) <= CHR_TE_DELTA; }
    static bool isLongMark(uint32_t d) {
        return rft_absd((int)d - (int)CHR_TE_LONG_A) <= CHR_TE_LONG_DELTA ||
               rft_absd((int)d - (int)CHR_TE_LONG_B) <= CHR_TE_LONG_DELTA;
    }

    static const uint8_t *xorTable() {
        static const uint8_t t[16] = {0x0F, 0x02, 0x40, 0x0C, 0x30, 0x0E, 0x70, 0x08,
                                      0x10, 0x0A, 0x50, 0xF4, 0x2F, 0xF6, 0x6F, 0xF0};
        return t;
    }

    static void transformBlock(const uint8_t in[9], uint8_t out[9], uint32_t key, uint8_t button) {
        uint8_t mask = xorTable()[key & 0x0FU];
        if (button == 1U) mask ^= (key & 1U) ? 0xF0U : 0x0FU;
        for (size_t i = 0; i < 9; i++) out[i] = in[i] ^ mask;
    }

    void reset() {
        decodeData = 0;
        decodeCount = 0;
        packetBitCount = 0;
        teLast = 0;
        data2 = 0;
        step = Reset;
    }

    void decodePacket() {
        uint8_t key[8];
        uint8_t encoded[9];
        uint8_t decoded[9];
        const uint16_t key2 = data2;

        u64_to_bytes_be(decodeData, key);
        const uint8_t seed = reverse6(key[0] >> 2U);
        const uint8_t b1_xor_b6 = key[6] ^ key[1];
        const bool msbSet = (key[0] & 0x80U) != 0U;

        bool checkOk = false;
        uint8_t decodedButton = 1U;

        if (msbSet) {
            const uint8_t key2Low = (uint8_t)(key2 & 0xFFU);
            checkOk = (key[1] == key[5]) && (b1_xor_b6 == 0x62U);
            decodedButton = (((uint8_t)(key2Low ^ key[4])) == 0x10U) ? 2U : 1U;
        } else {
            if (((uint8_t)(key[1] ^ 0xC3U)) == key[5]) {
                if (b1_xor_b6 == 0x04U) {
                    checkOk = true;
                } else {
                    checkOk = (b1_xor_b6 == 0x08U);
                    if (checkOk) decodedButton = 2U;
                }
            } else {
                if (b1_xor_b6 == 0x08U) decodedButton = 2U;
            }
        }

        encoded[0] = key[1];
        encoded[1] = key[2];
        encoded[2] = key[3];
        encoded[3] = key[4];
        encoded[4] = key[5];
        encoded[5] = key[6];
        encoded[6] = key[7];
        encoded[7] = (uint8_t)(key2 >> 8U);
        encoded[8] = (uint8_t)(key2 & 0xFFU);
        transformBlock(encoded, decoded, seed, decodedButton);

        uint32_t cnt = 0;
        uint32_t snB = 0;
        bool plainB = false;
        if (seed & 1U) {
            plainB = true;
            snB = ((uint32_t)decoded[0] << 24U) | ((uint32_t)decoded[1] << 16U) |
                  ((uint32_t)decoded[2] << 8U) | (uint32_t)decoded[7];
        } else {
            cnt = ((uint32_t)decoded[0] << 24U) | ((uint32_t)decoded[1] << 16U) |
                  ((uint32_t)decoded[2] << 8U) | (uint32_t)decoded[3];
        }

        if (!checkOk) return;

        if (commits == 0) {
            frame.proto = RftProto::ChryslerV0;
            frame.key = decodeData;
            frame.key2 = key2;
            frame.seed = seed;
            frame.header = (uint8_t)(key[0] & 0x03U);
            frame.btn = decodedButton;
            frame.bits = CHR_DECODE_BIT_COUNT;
            frame.te = CHR_TE_SHORT;
            frame.crcOk = true;
        }

        // A real Chrysler frame carries two blocks: the counter block (even
        // seed) and the serial block (odd seed). Keep both so the encoder can
        // rebuild the pair exactly instead of guessing.
        if (plainB) {
            memcpy(frame.plainB, decoded, 9);
            frame.plainBPresent = true;
        } else {
            memcpy(frame.plainA, decoded, 9);
            frame.plainAPresent = true;
            frame.key = decodeData;
            frame.key2 = key2;
            frame.seed = seed;
            frame.header = (uint8_t)(key[0] & 0x03U);
            frame.btn = decodedButton;
            frame.cnt = cnt;
        }
        (void)snB;
        commits++;
    }

    void commit() {
        decodeCount = CHR_DECODE_BIT_COUNT;
        decodePacket();
    }

    void feed(bool level, uint32_t duration) {
        switch (step) {
        case Reset:
            if (level && isShort(duration)) {
                packetBitCount = 0;
                teLast = duration;
                step = Seek;
            }
            break;

        case Seek:
            if (level) {
                teLast = duration;
                break;
            }
            if (isLongMark(duration)) {
                if (isShort(teLast)) {
                    packetBitCount++;
                } else if (packetBitCount > 0x0F) {
                    data2 = 0;
                    step = Data;
                    decodeData = 1;
                    decodeCount = 1;
                } else {
                    packetBitCount = 0;
                    step = Seek;
                }
                break;
            }
            if (duration > CHR_TE_GAP && packetBitCount > 0x0F) {
                decodeData = 0;
                data2 = 0;
                decodeCount = 0;
                step = Data;
                break;
            }
            step = Reset;
            packetBitCount = 0;
            break;

        case Data: {
            if (level) {
                teLast = duration;
                break;
            }

            const uint8_t count = (uint8_t)decodeCount;
            if (duration > CHR_TE_GAP) {
                if (count > 0x4FU) {
                    commit();
                }
                step = Reset;
                packetBitCount = 0;
                break;
            }

            uint8_t bit_value = 0;
            if (teLast < CHR_TE_SHORT) {
                if (!isShort(teLast) || !isLongMark(duration)) {
                    if (count > 0x4FU) commit();
                    step = Reset;
                    packetBitCount = 0;
                    break;
                }
                bit_value = 1U;
            } else {
                if (teLast > 0x2EEU || !isLongMark(duration)) {
                    if (count > 0x4FU) commit();
                    step = Reset;
                    packetBitCount = 0;
                    break;
                }
                bit_value = isShort(teLast) ? 1U : 0U;
            }

            const uint8_t bit = bit_value ^ 1U;
            const uint8_t newCount = (uint8_t)(count + 1U);
            if (count <= 0x3FU) {
                decodeData = (decodeData << 1U) | bit;
                decodeCount = newCount;
                break;
            }

            data2 = (uint16_t)((data2 << 1U) | bit);
            decodeCount = newCount;
            if (newCount != CHR_DECODE_BIT_COUNT) break;

            commit();
            decodeData = 0;
            data2 = 0;
            decodeCount = 0;
            step = Reset;
            packetBitCount = 0;
            break;
        }

        default:
            step = Reset;
            break;
        }
    }
};

// ---------------------------------------------------------------------------
// Kia V1 -- 3 bursts of 90 header pulses + Manchester payload, 57 bits.
// ---------------------------------------------------------------------------

constexpr uint32_t KIA_TE_SHORT = 800;
constexpr uint32_t KIA_TE_LONG = 1600;
constexpr uint32_t KIA_TE_DELTA = 200;
constexpr uint16_t KIA_MIN_BITS = 57;
constexpr uint16_t KIA_CELLS = KIA_MIN_BITS * 2;

// The upstream ProtoPirate encoder and decoder for Kia do not agree with each
// other, and real Kia fobs were never verified there. This port uses a clean,
// self-consistent Manchester wire format and a direct cell collector instead
// of the generic Flipper Manchester state machine (which lags one bit).

struct KiaDecoder {
    enum Step { Reset = 0, CheckPreamble, Data } step = Reset;

    uint64_t decodeData = 0;
    uint32_t teLast = 0;
    uint16_t headerCount = 0;
    uint8_t cells[KIA_CELLS] = {0};
    uint16_t cellCount = 0;

    uint32_t commits = 0;
    RftFrame frame;

    static uint8_t crc4(const uint8_t *bytes, int count, uint8_t offset) {
        uint8_t crc = 0;
        for (int i = 0; i < count; i++) {
            uint8_t b = bytes[i];
            crc ^= ((b & 0x0F) ^ (b >> 4));
        }
        return (uint8_t)((crc + offset) & 0x0F);
    }

    void reset() {
        step = Reset;
        decodeData = 0;
        teLast = 0;
        headerCount = 0;
        cellCount = 0;
    }

    void checkRemote() {
        const uint32_t serial = (uint32_t)(decodeData >> 24);
        const uint8_t btn = (uint8_t)((decodeData >> 16) & 0xFF);
        const uint16_t cnt = (uint16_t)((((decodeData >> 4) & 0xF) << 8) |
                                        ((decodeData >> 8) & 0xFF));

        uint8_t charData[7];
        charData[0] = (uint8_t)(serial >> 24);
        charData[1] = (uint8_t)(serial >> 16);
        charData[2] = (uint8_t)(serial >> 8);
        charData[3] = (uint8_t)serial;
        charData[4] = btn;
        charData[5] = (uint8_t)(cnt & 0xFF);
        charData[6] = (uint8_t)((cnt >> 8) & 0xF);

        const uint8_t crc = crc4(charData, 7, 1);
        const bool crcCheck = crc == (uint8_t)(decodeData & 0xF);

        if (commits == 0) {
            frame.proto = RftProto::KiaV1;
            frame.key = decodeData;
            frame.serial = serial;
            frame.btn = btn;
            frame.cnt = cnt;
            frame.bits = KIA_MIN_BITS;
            frame.te = KIA_TE_SHORT;
            frame.crc = crc;
            frame.crcOk = crcCheck;
        }
        commits++;
    }

    void pushCell(bool level) {
        if (cellCount < KIA_CELLS) cells[cellCount++] = level ? 1U : 0U;
    }

    void finishFrame() {
        bool ok = true;
        uint64_t data = 0;
        for (uint16_t i = 0; i < KIA_MIN_BITS; i++) {
            const uint8_t first = cells[i * 2U];
            const uint8_t second = cells[i * 2U + 1U];
            if (first == second) {
                ok = false;
                break;
            }
            data = (data << 1) | first;
        }
        if (ok) {
            decodeData = data;
            checkRemote();
        }
        step = Reset;
        cellCount = 0;
    }

    void feed(bool level, uint32_t duration) {
        const bool sh = near(duration, KIA_TE_SHORT, KIA_TE_DELTA);
        const bool lg = near(duration, KIA_TE_LONG, KIA_TE_DELTA);

        switch (step) {
        case Reset:
            if (level && lg) {
                step = CheckPreamble;
                teLast = duration;
                headerCount = 0;
            }
            break;

        case CheckPreamble:
            if (!level) {
                if (lg && near(teLast, KIA_TE_LONG, KIA_TE_DELTA)) {
                    headerCount++;
                    teLast = duration;
                    break;
                }
                // Header lows are 2x te. A low short after a long header run
                // is the separator that starts the Manchester payload.
                if (sh && headerCount > 70) {
                    step = Data;
                    cellCount = 0;
                    break;
                }
                step = Reset;
                break;
            }
            teLast = duration;
            break;

        case Data:
            if (sh) {
                pushCell(level);
            } else if (lg) {
                pushCell(level);
                pushCell(level);
            } else {
                step = Reset;
                cellCount = 0;
                break;
            }
            if (cellCount >= KIA_CELLS) finishFrame();
            break;
        }
    }
};

// ---------------------------------------------------------------------------
// Fiat V2 -- 112-bit Manchester frame (224 cells), 14 raw bytes.
// ---------------------------------------------------------------------------

constexpr uint32_t FIAT_TE_SHORT = 210;
constexpr uint32_t FIAT_TE_LONG = 420;
constexpr uint32_t FIAT_TE_DELTA = 100;
constexpr uint32_t FIAT_BOUNDARY_MIN_US = 900;
constexpr uint8_t FIAT_WIRE_BITS = 112;
constexpr uint8_t FIAT_WIRE_BYTES = 14;
constexpr uint8_t FIAT_WIRE_CELLS = FIAT_WIRE_BITS * 2;

struct FiatDecoder {
    uint8_t cells[FIAT_WIRE_CELLS] = {0};
    uint16_t cellCount = 0;
    uint8_t rawData[FIAT_WIRE_BYTES] = {0};
    uint8_t lastRaw[FIAT_WIRE_BYTES] = {0};
    bool lastRawValid = false;

    uint32_t commits = 0;
    RftFrame frame;

    static bool isFca(const uint8_t raw[FIAT_WIRE_BYTES]) { return (raw[6] & 0xF0U) == 0xD0U; }

    static uint32_t uid(const uint8_t raw[FIAT_WIRE_BYTES]) {
        return ((uint32_t)raw[2] << 24U) | ((uint32_t)raw[3] << 16U) | ((uint32_t)raw[4] << 8U) |
               raw[5];
    }

    static uint32_t hop(const uint8_t raw[FIAT_WIRE_BYTES]) {
        if (isFca(raw)) {
            return ((uint32_t)raw[10] << 24U) | ((uint32_t)raw[11] << 16U) |
                   ((uint32_t)raw[12] << 8U) | raw[13];
        }
        return ((uint32_t)raw[9] << 24U) | ((uint32_t)raw[10] << 16U) |
               ((uint32_t)raw[11] << 8U) | raw[12];
    }

    static uint32_t counter(const uint8_t raw[FIAT_WIRE_BYTES]) {
        if (isFca(raw)) {
            const uint32_t rawCnt = ((uint32_t)raw[8] << 6U) | (uint32_t)(raw[9] >> 2U);
            return (~rawCnt) & 0x3FFFU;
        }
        const uint32_t rawCnt =
            ((uint32_t)(raw[7] & 0x3FU) << 5U) | (uint32_t)(raw[8] >> 3U);
        return (~rawCnt) & 0x7FFU;
    }

    static bool frameValid(const uint8_t raw[FIAT_WIRE_BYTES]) {
        if (raw[0] != 0x00U || raw[1] != 0x01U) return false;
        const uint8_t sel = raw[7] >> 6U;
        if (sel != 0x2U && sel != 0x3U && sel != 0x1U) return false;
        const uint32_t id = uid(raw);
        return id != 0U && id != UINT32_MAX;
    }

    void reset() {
        cellCount = 0;
        memset(cells, 0, sizeof(cells));
        memset(rawData, 0, sizeof(rawData));
        memset(lastRaw, 0, sizeof(lastRaw));
        lastRawValid = false;
    }

    void commit(const uint8_t raw[FIAT_WIRE_BYTES]) {
        if (!frameValid(raw)) return;
        if (lastRawValid && memcmp(lastRaw, raw, FIAT_WIRE_BYTES) == 0) return;

        memcpy(rawData, raw, FIAT_WIRE_BYTES);
        memcpy(lastRaw, raw, FIAT_WIRE_BYTES);
        lastRawValid = true;

        if (commits == 0) {
            frame.proto = RftProto::FiatV2;
            frame.serial = uid(raw);
            frame.btn = raw[7];
            frame.cnt = counter(raw);
            frame.hop = hop(raw);
            frame.key = ((uint64_t)frame.serial << 32U) | frame.hop;
            memcpy(frame.raw, raw, FIAT_WIRE_BYTES);
            frame.rawLen = FIAT_WIRE_BYTES;
            frame.bits = FIAT_WIRE_BITS;
            frame.te = FIAT_TE_SHORT;
            frame.crcOk = true;
        }
        commits++;
    }

    bool tryWindow(bool invert) {
        if (cellCount != FIAT_WIRE_CELLS) return false;

        uint8_t raw[FIAT_WIRE_BYTES] = {0};
        for (uint8_t bitIndex = 0; bitIndex < FIAT_WIRE_BITS; bitIndex++) {
            const uint8_t first = cells[bitIndex * 2U];
            const uint8_t second = cells[bitIndex * 2U + 1U];
            if (first == second) return false;

            bool bit = first != 0U;
            if (invert) bit = !bit;
            if (bit) raw[bitIndex >> 3U] |= (uint8_t)(1U << (7U - (bitIndex & 7U)));
        }

        const bool valid = frameValid(raw);
        if (valid) commit(raw);
        return valid;
    }

    void tryDecode() {
        if (tryWindow(false)) return;
        tryWindow(true);
    }

    void pushCell(bool level) {
        if (cellCount < FIAT_WIRE_CELLS) {
            cells[cellCount++] = level ? 1U : 0U;
        } else {
            memmove(cells, &cells[1], FIAT_WIRE_CELLS - 1U);
            cells[FIAT_WIRE_CELLS - 1U] = level ? 1U : 0U;
        }
        tryDecode();
    }

    bool feedDataPulse(bool level, uint32_t duration) {
        if (near(duration, FIAT_TE_SHORT, FIAT_TE_DELTA)) {
            pushCell(level);
            return true;
        }
        if (near(duration, FIAT_TE_LONG, FIAT_TE_DELTA)) {
            pushCell(level);
            pushCell(level);
            return true;
        }
        if (!level && duration >= FIAT_BOUNDARY_MIN_US) pushCell(false);
        cellCount = 0;
        memset(cells, 0, sizeof(cells));
        return false;
    }

    enum Step { Reset = 0, Data = 1 } step = Reset;

    void feed(bool level, uint32_t duration) {
        const bool shortPulse = near(duration, FIAT_TE_SHORT, FIAT_TE_DELTA);
        const bool longPulse = near(duration, FIAT_TE_LONG, FIAT_TE_DELTA);

        switch (step) {
        case Reset:
            if (shortPulse || longPulse) {
                cellCount = 0;
                memset(cells, 0, sizeof(cells));
                step = Data;
                feedDataPulse(level, duration);
            }
            break;
        case Data:
            if (!feedDataPulse(level, duration)) step = Reset;
            break;
        }
    }
};

} // namespace

const char *RftFrame::protocolName() const {
    switch (proto) {
    case RftProto::HondaV1: return "Honda V1";
    case RftProto::ChryslerV0: return "Chrysler V0";
    case RftProto::KiaV1: return "Kia V1";
    case RftProto::FiatV2: return "Fiat V2";
    default: return "Unknown";
    }
}

const char *RftFrame::buttonName() const {
    switch (proto) {
    case RftProto::HondaV1:
        switch (btn) {
        case 0: return "Unlock";
        case 8: return "Lock";
        case 9: return "Trunk";
        case 10: return "Panic";
        default: return "Unknown";
        }
    case RftProto::ChryslerV0:
        return btn == 2 ? "Unlock" : "Lock";
    case RftProto::KiaV1:
        switch (btn) {
        case 1: return "Close";
        case 2: return "Open";
        case 3: return "Boot";
        default: return "Unknown";
        }
    case RftProto::FiatV2:
        switch (btn >> 6) {
        case 2: return "Lock";
        case 3: return "Unlock";
        case 1: return "Trunk";
        default: return "Unknown";
        }
    default:
        return "Unknown";
    }
}

bool RftFrame::hasEncoder() const {
    return proto == RftProto::HondaV1 || proto == RftProto::ChryslerV0 || proto == RftProto::KiaV1;
}

bool rft_decode(const std::vector<int> &durations, RftFrame &out, RftProto only) {
    HondaDecoder honda;
    ChryslerDecoder chrysler;
    KiaDecoder kia;
    FiatDecoder fiat;

    honda.reset();
    chrysler.reset();
    kia.reset();
    fiat.reset();

    for (int d : durations) {
        const bool level = d > 0;
        const uint32_t us = rft_absd(d);
        if (us == 0) continue;

        if (only == RftProto::None || only == RftProto::HondaV1) honda.feed(level, us);
        if (only == RftProto::None || only == RftProto::ChryslerV0) chrysler.feed(level, us);
        if (only == RftProto::None || only == RftProto::KiaV1) kia.feed(level, us);
        if (only == RftProto::None || only == RftProto::FiatV2) fiat.feed(level, us);
    }

    // Pick the protocol that committed the most times in this buffer. A real
    // fob repeats its frame, so the correct protocol wins over a stray partial
    // match from another decoder.
    RftProto best = RftProto::None;
    uint32_t bestCount = 0;
    auto consider = [&](RftProto p, uint32_t count) {
        if (count > bestCount) {
            bestCount = count;
            best = p;
        }
    };
    consider(RftProto::HondaV1, honda.commits);
    consider(RftProto::ChryslerV0, chrysler.commits);
    consider(RftProto::KiaV1, kia.commits);
    consider(RftProto::FiatV2, fiat.commits);

    if (best == RftProto::None) return false;

    switch (best) {
    case RftProto::HondaV1: out = honda.frame; break;
    case RftProto::ChryslerV0: out = chrysler.frame; break;
    case RftProto::KiaV1: out = kia.frame; break;
    case RftProto::FiatV2: out = fiat.frame; break;
    default: return false;
    }
    return out.proto != RftProto::None;
}

String rft_summary(const RftFrame &frame) {
    String s;
    s += frame.protocolName();
    s += "  ";
    s += frame.buttonName();
    s += "\n";
    char buf[96];
    snprintf(buf, sizeof(buf), "Sn:%08lX  Cnt:%04lX\n", (unsigned long)frame.serial,
             (unsigned long)frame.cnt);
    s += buf;
    if (frame.proto == RftProto::FiatV2) {
        snprintf(buf, sizeof(buf), "Hop:%08lX  Btn:%02X\n", (unsigned long)frame.hop,
                 (unsigned)frame.btn);
        s += buf;
    } else if (frame.proto == RftProto::ChryslerV0) {
        snprintf(buf, sizeof(buf), "Seed:%02X  Btn:%02X\n", (unsigned)frame.seed,
                 (unsigned)frame.btn);
        s += buf;
    } else {
        snprintf(buf, sizeof(buf), "Key:%08lX%08lX\n", (unsigned long)(frame.key >> 32),
                 (unsigned long)(frame.key & 0xFFFFFFFFu));
        s += buf;
    }
    s += frame.hasEncoder() ? "Encoder: yes" : "Encoder: no (raw replay)";
    return s;
}
