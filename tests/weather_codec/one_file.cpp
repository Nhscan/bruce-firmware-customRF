#include "rf_weather_decoders.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static std::string load(const char *path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

int main(int argc, char **argv) {
    for (int a = 1; a < argc; a++) {
        const std::string raw = load(argv[a]);
        String text(raw.c_str());
        auto d = weather_parse_sub(text);
        std::vector<WeatherRepeat> reps;
        WeatherReading st;
        const bool ok = weather_decode(d, reps, st, 1);
        printf("\n%s\n  pulses=%u\n", argv[a], (unsigned)d.size());
        if (ok) {
            printf(
                "  DECODED %s id=%d ch=%d temp=%.1f hum=%d(%d) wind=%.1f rain=%.1f\n"
                "  raw=",
                st.model.c_str(),
                st.id,
                st.channel,
                st.tempC,
                st.hasHumidity ? st.humidity : -1,
                st.hasHumidity,
                st.windKph,
                st.rainMm);
            for (int i = 0; i < (st.rawBits + 7) / 8; i++) printf("%02X ", st.raw[i]);
            printf("\n");
        } else {
            printf("  NO DECODE\n  %s\n", weather_diagnose(d).c_str());
        }
    }
    return 0;
}
