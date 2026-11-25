#ifndef RF_WEATHER_DECODERS_H
#define RF_WEATHER_DECODERS_H

// Pure weather-protocol decoders used by the Weather Station app.
//
// Kept free of display/radio dependencies so the same code can be exercised
// on the host against Flipper `.sub` captures (tests/weather_codec).
//
// Supported:
//   - AcuRite 606TX (32-bit PPM, LFSR checksum)
//   - AcuRite TXR family: 592TXR/Tower (56), 3n1/5n1 (64), 6045M (72),
//     Atlas (80) -- PWM 220/408 us, checksum + even parity
//   - Nexus / Nexus-TH (36-bit PPM, sync nibble, repeated frames)
//   - Prologue (36-bit PPM, repeated frames)
//   - LaCrosse TX141TH-Bv2/Bv3 (40-bit PWM, LFSR checksum)
#include <Arduino.h>
#include <stdint.h>
#include <vector>

struct WeatherReading {
    String model;
    int id = 0;
    int channel = 0;
    bool battery = true;
    float tempC = 0.0f;
    bool hasTemp = true;
    bool hasHumidity = false;
    int humidity = 0;
    bool hasWind = false;
    float windKph = 0.0f;
    bool hasRain = false;
    float rainMm = 0.0f;
    uint8_t raw[10] = {0};
    int rawBits = 0;
};

struct WeatherRepeat {
    int profile = 0;
    uint64_t frame = 0;
    int count = 0;
    uint32_t lastMs = 0;
};

// Decode one capture. `repeats` persists across calls for the protocols that
// have no checksum (Nexus/Prologue); `minRepeats` is 3 for live listening and
// 1 when decoding a saved file. Returns true and fills `out` on a match.
bool weather_decode(
    const std::vector<int> &durations,
    std::vector<WeatherRepeat> &repeats,
    WeatherReading &out,
    int minRepeats = 3
);

// Short human-readable extraction summary for the diagnostics screen/serial.
String weather_diagnose(const std::vector<int> &durations);

// Parse a Flipper/Bruce `.sub` text (all RAW_Data lines) into signed
// durations (positive = high, negative = low, microseconds).
std::vector<int> weather_parse_sub(const String &text);

// Protocol names in priority order (for diagnostics/UI).
const char *weather_protocol_name(uint8_t index);

#endif
