// RF Tools for Bruce -- automotive key-fob decoders/encoders and the shared
// RF Tools menu.
//
// The four protocols here (Honda V1, Chrysler V0, Kia V1, Fiat V2) are ports
// of the ProtoPirate implementations used by the Flipper "RF Tools" app.
// They are deliberately kept OUT of Bruce's generic OOK registry: they are
// framed/PWM protocols with their own timing rules, and mixing them into the
// registry would make every ordinary 433 MHz decode slower and noisier.
//
// Decoding is done over a captured buffer of signed pulse durations
// (positive = high, negative = low, microseconds) exactly like the rest of
// Bruce's RF module produces (RfRxSession::poll / rf_build_raw).
#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <vector>

enum class RftProto : uint8_t {
    None = 0,
    HondaV1,
    ChryslerV0,
    KiaV1,
    FiatV2,
};

struct RftFrame {
    RftProto proto = RftProto::None;

    uint32_t serial = 0;
    uint32_t btn = 0;
    uint32_t cnt = 0;
    uint32_t hop = 0;

    uint64_t key = 0;   // Honda/Kia: full data word. Chrysler: encoded block A.
    uint16_t key2 = 0;  // Chrysler data_2

    uint8_t raw[14] = {0}; // Fiat wire bytes
    uint8_t rawLen = 0;

    uint8_t plainA[9] = {0};
    uint8_t plainB[9] = {0};
    bool plainAPresent = false;
    bool plainBPresent = false;

    uint8_t seed = 0;
    uint8_t header = 0; // Chrysler plain header (2 bits)
    uint8_t crc = 0;
    bool crcOk = false;
    uint8_t bits = 0;
    int te = 0;
    int repeat = 3;

    const char *protocolName() const;
    const char *buttonName() const;
    bool hasEncoder() const;
};

// Decode one capture. When `only` is not None, only that protocol is tried.
// Returns true and fills `out` when a frame was accepted.
bool rft_decode(const std::vector<int> &durations, RftFrame &out, RftProto only = RftProto::None);

// Build a transmittable signed-duration buffer from a decoded frame.
// Fiat V2 has no encoder (raw replay only), so this returns false for it.
bool rft_encode(RftFrame &frame, std::vector<int> &out);

// Flipper-compatible `.key` snapshot text.
String rft_key_text(const RftFrame &frame, float freqMhz);
bool rft_parse_key(const String &text, RftFrame &frame);

// Human-readable decode summary (multi-line, tft friendly).
String rft_summary(const RftFrame &frame);

// --- Car Keys app ---------------------------------------------------------
void rf_car_keys();

// Shared radio settings (also used by the RF Tools menu).
struct RftSettings {
    float freq = 433.92f;
    float bw = 135.0f;
    int agc = 3;
    int repeat = 3;
    int counterIncr = 1;
    bool autoSave = true;
};

extern RftSettings rftSettings;
void rft_settings_load();
void rft_settings_save();
void rft_radio_settings_menu();

// --- RF Tools menu --------------------------------------------------------
void rf_tools_menu();
