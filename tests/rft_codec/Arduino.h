#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

// Minimal Arduino String stand-in for host-side codec tests only.
class String {
public:
    std::string s;

    String() {}
    String(const char *p) : s(p ? p : "") {}
    String(const std::string &v) : s(v) {}
    String(int v) { s = std::to_string(v); }
    String(unsigned int v) { s = std::to_string(v); }
    String(long v) { s = std::to_string(v); }
    String(unsigned long v) { s = std::to_string(v); }
    String(float v, int digits = 2) {
        char b[64];
        snprintf(b, sizeof(b), "%.*f", digits, v);
        s = b;
    }

    char operator[](size_t i) const { return i < s.size() ? s[i] : '\0'; }
    String &operator+=(const String &o) {
        s += o.s;
        return *this;
    }
    String &operator+=(const char *o) {
        s += o;
        return *this;
    }
    String &operator+=(char o) {
        s += o;
        return *this;
    }
    bool operator==(const String &o) const { return s == o.s; }

    const char *c_str() const { return s.c_str(); }
    size_t length() const { return s.size(); }
    void reserve(size_t n) { s.reserve(n); }

    int indexOf(char c, int from = 0) const {
        const size_t p = s.find(c, (size_t)(from < 0 ? 0 : from));
        return p == std::string::npos ? -1 : (int)p;
    }
    int indexOf(const char *c, int from = 0) const {
        const size_t p = s.find(c, (size_t)(from < 0 ? 0 : from));
        return p == std::string::npos ? -1 : (int)p;
    }
    String substring(int from) const {
        if (from < 0) from = 0;
        if ((size_t)from > s.size()) return String();
        return String(s.substr((size_t)from));
    }
    String substring(int from, int to) const {
        if (from < 0) from = 0;
        if (to < from) return String();
        if ((size_t)from > s.size()) return String();
        const size_t n = std::min((size_t)to, s.size()) - (size_t)from;
        return String(s.substr((size_t)from, n));
    }
    void trim() {
        size_t a = 0, b = s.size();
        while (a < b && (unsigned char)s[a] <= ' ') a++;
        while (b > a && (unsigned char)s[b - 1] <= ' ') b--;
        s = s.substr(a, b - a);
    }
    bool startsWith(const char *p) const { return s.rfind(p, 0) == 0; }
    bool endsWith(const char *p) const {
        const size_t n = strlen(p);
        return s.size() >= n && s.compare(s.size() - n, n, p) == 0;
    }
    void remove(int idx) {
        if (idx >= 0 && (size_t)idx < s.size()) s.erase((size_t)idx);
    }
};
