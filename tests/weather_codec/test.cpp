#include "rf_weather_decoders.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_fail = 0;

static void check(bool cond, const std::string &what) {
    printf("%s %s\n", cond ? "PASS" : "FAIL", what.c_str());
    if (!cond) g_fail++;
}

static std::string load(const char *path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static WeatherReading decodeFile(const char *path, bool &ok) {    const std::string raw = load(path);
    String text(raw.c_str());
    auto d = weather_parse_sub(text);
    std::vector<WeatherRepeat> reps;
    WeatherReading st;
    ok = weather_decode(d, reps, st, 1);
    printf("  %-55s pulses=%-5u -> ", path, (unsigned)d.size());
    if (ok) {
        printf(
            "%s id=%d ch=%d temp=%.1f hum=%d(%d) wind=%.1f rain=%.1f\n",
            st.model.c_str(),
            st.id,
            st.channel,
            st.tempC,
            st.hasHumidity ? st.humidity : -1,
            st.hasHumidity,
            st.windKph,
            st.rainMm);
    } else {
        printf("NO DECODE  [%s]\n", weather_diagnose(d).c_str());
    }
    return st;
}

static uint8_t lfsr8(const uint8_t *msg, int n, uint8_t gen, uint8_t key) {
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

static void testAcuRite606() {
    uint8_t b[4] = {0x84, 0x80, 0x83, 0};
    b[3] = lfsr8(b, 3, 0x98, 0xF1);

    std::vector<int> d;
    d.push_back(-8000);
    for (int i = 0; i < 32; i++) {
        const int bit = (b[i >> 3] >> (7 - (i & 7))) & 1;
        d.push_back(500);
        d.push_back(bit ? -4000 : -2000);
    }

    std::vector<WeatherRepeat> reps;
    WeatherReading st;
    const bool ok = weather_decode(d, reps, st, 1);
    printf("  606 synthetic: ");
    if (ok) printf("%s id=%d ch=%d temp=%.1f\n", st.model.c_str(), st.id, st.channel, st.tempC);
    else printf("NO DECODE [%s]\n", weather_diagnose(d).c_str());
    check(
        ok && st.model == "AcuRite-606TX" && st.id == 0x84 && st.channel == 1 &&
            st.tempC > 13.0 && st.tempC < 13.2,
        "AcuRite 606TX synthetic");
}

int main(int argc, char **argv) {
    const char *root = (argc > 1) ? argv[1] : "C:\\Users\\admin\\Desktop\\Weather_stations";

    bool ok;
    char path[512];

    testAcuRite606();

    snprintf(path, sizeof(path), "%s\\Nexus-TH\\nexus-th_raw.sub", root);
    WeatherReading st = decodeFile(path, ok);
    check(ok && st.model == "Nexus-TH" && st.id == 71 && st.tempC > 29.4 && st.tempC < 29.6 &&
              st.humidity == 40,
          "Nexus sample");

    snprintf(path, sizeof(path), "%s\\VITEK_VT-3531\\T0.sub", root);
    st = decodeFile(path, ok);
    check(ok && st.model.startsWith("Nexus") && st.id == 71 && !st.hasHumidity, "VITEK T0 (hum sentinel)");

    snprintf(path, sizeof(path), "%s\\VITEK_VT-3531\\T1.sub", root);
    st = decodeFile(path, ok);
    check(ok && st.model == "Nexus-TH" && st.id == 71 && st.humidity == 39, "VITEK T1");

    snprintf(path, sizeof(path), "%s\\LaCrosse-TX141TH-BV2\\LaCrosse-TX141TH-BV2-raw.sub", root);
    st = decodeFile(path, ok);
    check(ok && st.model.startsWith("LaCrosse") && st.id == 18 && st.humidity == 50 &&
              st.tempC > 16.0 && st.tempC < 16.2,
          "LaCrosse sample");

    snprintf(path, sizeof(path), "%s\\Acurite_02077M\\RAW_2022.10.21-18.04.09.sub", root);
    st = decodeFile(path, ok);
    check(ok && st.model == "AcuRite-Tower" && st.id == 6315 && st.humidity == 83, "AcuRite 02077M small");

    snprintf(path, sizeof(path), "%s\\Acurite_02077M\\RAW_2022.10.21-18.01.44.sub", root);
    st = decodeFile(path, ok);
    check(ok && st.model == "AcuRite-Tower" && st.id == 6315 && st.humidity == 79 &&
              st.tempC > 1.8 && st.tempC < 2.0,
          "AcuRite 02077M large");

    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL PASS", g_fail);
    return g_fail ? 1 : 0;
}
