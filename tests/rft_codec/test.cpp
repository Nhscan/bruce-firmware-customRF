#include "rft.h"

#include <cstdio>
#include <cstring>
#include <vector>

static int g_fail = 0;

static void check(bool cond, const char *what) {
    printf("%s %s\n", cond ? "PASS" : "FAIL", what);
    if (!cond) g_fail++;
}

static void checkEq(uint32_t a, uint32_t b, const char *what) {
    if (a == b) {
        printf("PASS %s (0x%lX)\n", what, (unsigned long)a);
    } else {
        printf("FAIL %s: got 0x%lX expected 0x%lX\n", what, (unsigned long)a, (unsigned long)b);
        g_fail++;
    }
}

static void emit(std::vector<int> &v, bool level, int us) {
    v.push_back(level ? us : -us);
}

static void test_honda() {
    RftFrame in;
    in.proto = RftProto::HondaV1;
    in.serial = 0x02345678;
    in.btn = 8;
    in.cnt = 0x1234;
    in.bits = 68;

    std::vector<int> pulses;
    bool ok = rft_encode(in, pulses);
    check(ok, "Honda encode");
    check(pulses.size() > 100, "Honda pulse count");
    printf("  Honda pulses=%u\n", (unsigned)pulses.size());

    RftFrame out;
    ok = rft_decode(pulses, out);
    check(ok, "Honda decode");
    check(out.proto == RftProto::HondaV1, "Honda proto");
    checkEq(out.serial, 0x02345678, "Honda serial");
    checkEq(out.btn, 8, "Honda button");
    checkEq(out.cnt, 0x1234, "Honda counter");
}

static void test_kia() {
    RftFrame in;
    in.proto = RftProto::KiaV1;
    in.serial = 0x12345678;
    in.btn = 2;
    in.cnt = 0x123;
    in.bits = 57;

    std::vector<int> pulses;
    bool ok = rft_encode(in, pulses);
    check(ok, "Kia encode");
    printf("  Kia pulses=%u\n", (unsigned)pulses.size());

    RftFrame out;
    ok = rft_decode(pulses, out);
    check(ok, "Kia decode");
    check(out.proto == RftProto::KiaV1, "Kia proto");
    checkEq(out.serial, 0x12345678, "Kia serial");
    checkEq(out.btn, 2, "Kia button");
    checkEq(out.cnt, 0x123, "Kia counter");
    check(out.crcOk, "Kia CRC");
}

static void test_chrysler() {
    // Synthesize a valid 80-bit packet directly, using the MSB-set layout:
    // key[1] == key[5], key[6] ^ key[1] == 0x62, key2_low ^ key[4] == 0x10.
    uint8_t payload[10] = {0x80, 0xAA, 0x11, 0x22, 0x10, 0xAA, 0xC8, 0x33, 0x00, 0x00};

    std::vector<int> pulses;
    for (int i = 0; i < 24; i++) {
        emit(pulses, true, 0x12C);
        emit(pulses, false, 0xE74);
    }
    emit(pulses, true, 0x12C);
    emit(pulses, false, 0x3CF0);

    for (int bit = 0; bit < 80; bit++) {
        const int value = (payload[bit >> 3] >> (7 - (bit & 7))) & 1;
        emit(pulses, true, value ? 0x258 : 0x12C);
        emit(pulses, false, value ? 0xD48 : 0xE74);
    }
    emit(pulses, true, 0x12C);
    emit(pulses, false, 0x3CF0);

    RftFrame out;
    const bool ok = rft_decode(pulses, out);
    check(ok, "Chrysler decode");
    if (ok) {
        check(out.proto == RftProto::ChryslerV0, "Chrysler proto");
        checkEq(out.btn, 2, "Chrysler button");
        checkEq(out.seed, 1, "Chrysler seed");
        uint64_t expected = 0;
        for (int i = 0; i < 8; i++) expected = (expected << 8) | payload[i];
        check(out.key == expected, "Chrysler key");
        check(out.plainBPresent, "Chrysler plain B captured");
    }
}

static void test_fiat() {
    uint8_t raw[14] = {0x00, 0x01, 0xC9, 0xC9, 0x13, 0xF2, 0xD0, 0xFC,
                       0x00, 0x00, 0xDF, 0xA5, 0xC9, 0x41};

    std::vector<int> pulses;
    for (int bit = 0; bit < 112; bit++) {
        const int b = (raw[bit >> 3] >> (7 - (bit & 7))) & 1;
        // Manchester cells: bit 1 -> 1,0 ; bit 0 -> 0,1
        emit(pulses, b != 0, 210);
        emit(pulses, b == 0, 210);
    }

    RftFrame out;
    const bool ok = rft_decode(pulses, out);
    check(ok, "Fiat decode");
    if (ok) {
        check(out.proto == RftProto::FiatV2, "Fiat proto");
        checkEq(out.serial, 0xC9C913F2, "Fiat UID");
        checkEq(out.btn, 0xFC, "Fiat button");
        checkEq(out.hop, 0xDFA5C941, "Fiat hop");
        checkEq(out.cnt, 0x3FFF, "Fiat counter");
        check(out.rawLen == 14 && memcmp(out.raw, raw, 14) == 0, "Fiat raw bytes");
    }
}

static void test_fiat_merged() {
    uint8_t raw[14] = {0x00, 0x01, 0xC9, 0xC9, 0x13, 0xF2, 0xD0, 0xFC,
                       0x00, 0x00, 0xDF, 0xA5, 0xC9, 0x41};

    std::vector<int> cells;
    for (int bit = 0; bit < 112; bit++) {
        const int b = (raw[bit >> 3] >> (7 - (bit & 7))) & 1;
        cells.push_back(b);
        cells.push_back(!b);
    }
    // Merge runs of the same level into long pulses, exactly like the air
    // signal / RMT receiver would.
    std::vector<int> pulses;
    for (int c : cells) {
        const bool level = c != 0;
        const int us = 210;
        if (!pulses.empty() && (pulses.back() > 0) == level) {
            pulses.back() += level ? us : -us;
        } else {
            pulses.push_back(level ? us : -us);
        }
    }

    RftFrame out;
    check(rft_decode(pulses, out) && out.proto == RftProto::FiatV2, "Fiat merged decode");
    checkEq(out.serial, 0xC9C913F2, "Fiat merged UID");
}

static void test_chrysler_encode_smoke() {
    RftFrame in;
    in.proto = RftProto::ChryslerV0;
    in.seed = 4;
    in.header = 1;
    in.btn = 1;
    in.cnt = 4;
    for (int i = 0; i < 9; i++) {
        in.plainA[i] = (uint8_t)(0x10 + i);
        in.plainB[i] = (uint8_t)(0x20 + i);
    }
    in.plainAPresent = true;
    in.plainBPresent = true;

    std::vector<int> pulses;
    check(rft_encode(in, pulses), "Chrysler encode");
    check(pulses.size() > 300, "Chrysler pulse count");
    printf("  Chrysler pulses=%u key=%016llX k2=%04X\n", (unsigned)pulses.size(),
           (unsigned long long)in.key, (unsigned)in.key2);
}

static void test_honda_jitter() {
    RftFrame in;
    in.proto = RftProto::HondaV1;
    in.serial = 0x02345678;
    in.btn = 8;
    in.cnt = 0x1234;

    std::vector<int> pulses;
    rft_encode(in, pulses);
    unsigned seed = 12345;
    for (auto &p : pulses) {
        seed = seed * 1103515245u + 12345u;
        const int jitter = (int)((seed >> 16) % 201) - 100;
        p += (p > 0) ? jitter : -jitter;
    }

    RftFrame out;
    check(rft_decode(pulses, out) && out.proto == RftProto::HondaV1, "Honda jitter decode");
    checkEq(out.serial, 0x02345678, "Honda jitter serial");
}


static void test_keyfile() {
    RftFrame in;
    in.proto = RftProto::HondaV1;
    in.serial = 0x02345678;
    in.btn = 8;
    in.cnt = 0x1234;
    in.key = 0x1234567890ABCDEFULL;
    in.bits = 68;
    in.te = 1000;

    const String text = rft_key_text(in, 433.92f);
    printf("--- key file ---\n%s---------------\n", text.c_str());

    RftFrame out;
    check(rft_parse_key(text, out), "key parse");
    check(out.proto == RftProto::HondaV1, "key proto");
    checkEq(out.serial, in.serial, "key serial");
    checkEq(out.btn, in.btn, "key button");
    checkEq(out.cnt, in.cnt, "key counter");
    check(out.key == in.key, "key value");
}

int main() {
    test_honda();
    test_kia();
    test_chrysler();
    test_fiat();
    test_fiat_merged();
    test_chrysler_encode_smoke();
    test_honda_jitter();
    test_keyfile();
    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
