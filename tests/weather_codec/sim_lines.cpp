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

static std::vector<int> parse_line(const std::string &line) {
    std::vector<int> out;
    int i = 9;
    while (i < (int)line.size()) {
        while (i < (int)line.size() && (line[i] == ' ' || line[i] == '\t' || line[i] == '\r')) i++;
        if (i >= (int)line.size()) break;
        int sign = 1;
        if (line[i] == '-') { sign = -1; i++; }
        else if (line[i] == '+') i++;
        if (i >= (int)line.size() || line[i] < '0' || line[i] > '9') {
            while (i < (int)line.size() && line[i] != ' ') i++;
            continue;
        }
        long v = 0;
        while (i < (int)line.size() && line[i] >= '0' && line[i] <= '9') v = v * 10 + (line[i] - '0'), i++;
        out.push_back((int)(sign * v));
    }
    return out;
}

int main(int argc, char **argv) {
    for (int a = 1; a < argc; a++) {
        const std::string raw = load(argv[a]);
        std::vector<std::vector<int>> lines;
        {
            std::istringstream in(raw);
            std::string line;
            while (std::getline(in, line)) {
                if (line.rfind("RAW_Data:", 0) == 0) lines.push_back(parse_line(line));
            }
        }

        printf("\n%s\n  RAW_Data lines=%u\n", argv[a], (unsigned)lines.size());
        std::vector<WeatherRepeat> reps;
        std::vector<int> carry;
        bool found = false;
        for (size_t li = 0; li < lines.size(); li++) {
            std::vector<int> &ln = lines[li];
            const size_t cap = ln.size() < 4096 ? ln.size() : 4096; // RMT buffer cap
            std::vector<int> window;
            window.reserve(carry.size() + cap);
            window.insert(window.end(), carry.begin(), carry.end());
            window.insert(window.end(), ln.begin(), ln.begin() + cap);

            WeatherReading st;
            const bool ok = weather_decode(window, reps, st, 3);
            printf(
                "  line %2u: len=%-6u captured=%-5u %s",
                (unsigned)li,
                (unsigned)ln.size(),
                (unsigned)cap,
                ok ? "DECODED " : "no");
            if (ok) {
                printf("%s id=%d temp=%.1f hum=%d\n", st.model.c_str(), st.id, st.tempC,
                       st.hasHumidity ? st.humidity : -1);
                found = true;
                break;
            }
            printf("\n");
            const size_t keep = 512;
            if (window.size() > keep) carry.assign(window.end() - keep, window.end());
            else carry = window;
        }
        if (!found) printf("  NO DECODE ACROSS LINES\n");
    }
    return 0;
}
