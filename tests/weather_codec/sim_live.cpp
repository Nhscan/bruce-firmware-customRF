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
        std::vector<int> carry;
        bool found = false;
        WeatherReading st;
        int chunks = 0;

        for (size_t pos = 0; pos < d.size(); pos += 4096) {
            const size_t end = (pos + 4096 < d.size()) ? pos + 4096 : d.size();
            std::vector<int> window;
            window.reserve(carry.size() + (end - pos));
            window.insert(window.end(), carry.begin(), carry.end());
            window.insert(window.end(), d.begin() + pos, d.begin() + end);
            chunks++;
            if (weather_decode(window, reps, st, 3)) {
                found = true;
                break;
            }
            const size_t keep = 512;
            if (window.size() > keep) carry.assign(window.end() - keep, window.end());
            else carry = window;
        }

        printf("\n%s\n  pulses=%u chunks=%d\n", argv[a], (unsigned)d.size(), chunks);
        if (found) {
            printf(
                "  DECODED at chunk %d: %s id=%d temp=%.1f hum=%d\n",
                chunks,
                st.model.c_str(),
                st.id,
                st.tempC,
                st.hasHumidity ? st.humidity : -1);
        } else {
            printf("  NO DECODE after all chunks\n");
        }
    }
    return 0;
}
