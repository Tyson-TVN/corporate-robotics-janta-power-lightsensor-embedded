// Minimal fake Arduino/ESP32 layer so the real .ino can run on a desktop for
// tests. Not used on the board.
#pragma once
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstddef>
#include <string>
#include <algorithm>
#include <map>
#include <vector>
using std::max; using std::min;
#ifndef ARDUINO_ARCH_ESP32
#define ARDUINO_ARCH_ESP32 1
#endif
#define F(x) (x)
#define INPUT 0
#define ADC_11db 3

extern unsigned long mock_ms;
extern int mock_adc[40];
inline unsigned long millis() { return mock_ms; }
inline void delay(unsigned long ms) { mock_ms += ms; }
inline int analogRead(int pin) { return mock_adc[pin]; }
inline void analogReadResolution(int) {}
inline void analogSetAttenuation(int) {}
inline void pinMode(int, int) {}

struct MockSerial {
  std::string in, out;
  void begin(unsigned long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; int c = (unsigned char)in[0]; in.erase(0, 1); return c; }
  void print(const char* s) { out += s; }
  void print(char c) { out += c; }
  void print(int v) { out += std::to_string(v); }
  void print(unsigned long v) { out += std::to_string(v); }
  void print(double v, int d = 2) { char b[48]; std::snprintf(b, sizeof b, "%.*f", d, v); out += b; }
  void println() { out += "\n"; }
  template <class T> void println(T v) { print(v); out += "\n"; }
  void println(double v, int d) { print(v, d); out += "\n"; }
};
extern MockSerial Serial;
