// Fake ESP32 Preferences (flash key/value store) for desktop tests.
#pragma once
#include "Arduino.h"
extern std::map<std::string, std::vector<unsigned char>> mock_nvs;
class Preferences {
 public:
  bool begin(const char*, bool) { return true; }
  void end() {}
  size_t putBytes(const char* k, const void* p, size_t n) {
    auto c = (const unsigned char*)p; mock_nvs[k].assign(c, c + n); return n;
  }
  size_t getBytes(const char* k, void* p, size_t n) {
    auto it = mock_nvs.find(k);
    if (it == mock_nvs.end() || it->second.size() != n) return 0;
    std::memcpy(p, it->second.data(), n); return n;
  }
  bool remove(const char* k) { return mock_nvs.erase(k) > 0; }
};
