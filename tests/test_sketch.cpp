// Runs the REAL JantaPower_SunSensor.ino on a desktop with fake hardware.
// Checks DEMO scenarios, the LIVE calibration flow, and save/load, and writes
// a demo transcript to DEMO_OUTPUT.txt.
//
// Build and run (from /tests):
//   g++ -std=c++17 -Imock -I../firmware/JantaPower_SunSensor -x c++ test_sketch.cpp -o test_sketch && ./test_sketch
#include "Arduino.h"
unsigned long mock_ms = 0;
int mock_adc[40] = {0};
MockSerial Serial;
std::map<std::string, std::vector<unsigned char>> mock_nvs;

#include "../firmware/JantaPower_SunSensor/JantaPower_SunSensor.ino"

static int passed = 0, failed = 0;
#define CHECK(cond) do { if (cond) passed++; else { failed++; std::printf("FAIL line %d: %s\n", __LINE__, #cond); } } while (0)

static void run(unsigned long ms) { for (unsigned long i = 0; i < ms; i++) { mock_ms++; loop(); } }
static void cmd(const char* c) { Serial.in += c; Serial.in += "\n"; loop(); }
static bool near(float a, float b, float t) { return !std::isnan(a) && std::fabs(a - b) <= t; }

// Fake head: right/up light gives bigger counts (different "real" response
// than the demo model on purpose, with a mismatched channel).
static void setLight(float xd, float yd, float base) {
  const float D2R = 3.14159265f / 180.0f;
  float sx = 0.9f * std::tan(xd * D2R), sy = 0.9f * std::tan(yd * D2R);
  mock_adc[PIN_TL] = (int)(base * (1 - sx + sy) * 0.92f);  // TL is 8% weak
  mock_adc[PIN_TR] = (int)(base * (1 + sx + sy));
  mock_adc[PIN_BL] = (int)(base * (1 - sx - sy));
  mock_adc[PIN_BR] = (int)(base * (1 + sx - sy));
}

int main() {
  std::string transcript;
  setup();
  run(2000);
  CHECK(status == Status::NEAR_NORMAL_ESTIMATE && near(xDeg, 0, 0.5f) && near(yDeg, 0, 0.5f));

  struct { int n; Status s; float x, y; } cases[] = {
      {1, Status::ANGLE_ESTIMATE, 20, 0}, {2, Status::ANGLE_ESTIMATE, -10, 10},
      {3, Status::LOW_LIGHT, NAN, NAN},   {4, Status::SATURATED, NAN, NAN},
      {5, Status::OUT_OF_RANGE, NAN, NAN}, {6, Status::CHANNEL_FAULT, NAN, NAN},
      {0, Status::NEAR_NORMAL_ESTIMATE, 0, 0}};
  for (auto& c : cases) {
    char b[16]; std::snprintf(b, sizeof b, "demo %d", c.n);
    cmd(b); run(2500);
    CHECK(status == c.s);
    if (std::isnan(c.x)) CHECK(!angleValid && std::isnan(xDeg));
    else CHECK(angleValid && near(xDeg, c.x, 0.6f) && near(yDeg, c.y, 0.6f));
  }
  transcript += Serial.out; Serial.out.clear();

  // LIVE: uncalibrated -> no angles, calibration refused in DEMO
  cmd("demo 1"); cmd("cal x 10");
  CHECK(Serial.out.find("only work in LIVE") != std::string::npos);
  cmd("live"); setLight(0, 0, 900); run(2000);
  CHECK(status == Status::UNCALIBRATED && !angleValid && std::isnan(xDeg));

  // Calibrate: balance at zero, then -20/0/+20 on each axis
  cmd("balance"); run(2000);
  const float pts[] = {-20, 0, 20};
  for (float p : pts) { setLight(p, 0, 900); run(2000); char b[24]; std::snprintf(b, sizeof b, "cal x %.0f", p); cmd(b); }
  for (float p : pts) { setLight(0, p, 900); run(2000); char b[24]; std::snprintf(b, sizeof b, "cal y %.0f", p); cmd(b); }
  CHECK(liveCal.x.ready() && liveCal.y.ready());

  // Check an angle NOT used for calibration (interpolation error expected
  // because the fake response is non-linear and only 3 points were used)
  setLight(10, 0, 900); run(2000);
  CHECK(angleValid && near(xDeg, 10, 1.5f) && near(yDeg, 0, 0.5f));
  setLight(30, 0, 900); run(2000);
  CHECK(status == Status::OUT_OF_RANGE && !angleValid);

  // Save, clear, load
  cmd("save"); CHECK(Serial.out.find("# saved") != std::string::npos);
  Calibration before = liveCal;
  liveCal = Calibration();
  cmd("load");
  CHECK(liveCal.x.n == before.x.n && liveCal.y.n == before.y.n);
  mock_nvs["cal"][10] ^= 0xFF;  // corrupt saved data
  cmd("load"); CHECK(Serial.out.find("no valid saved") != std::string::npos);
  cmd("show");
  transcript += Serial.out;

  // millis() wrap-around keeps sampling
  mock_ms = 0xFFFFFFFFUL - 300; lastSampleMs = mock_ms; lastReportMs = mock_ms;
  setLight(0, 0, 900); run(3000);
  CHECK(angleValid);

  FILE* f = std::fopen("DEMO_OUTPUT.txt", "w");
  if (f) { std::fputs(transcript.c_str(), f); std::fclose(f); }
  std::printf("%d passed, %d failed\n", passed, failed);
  return failed ? 1 : 0;
}
