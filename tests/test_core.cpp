// Desktop unit tests for SunSensorCore.h (no Arduino needed).
// Build and run:  g++ -std=c++17 -Wall -I../firmware/JantaPower_SunSensor test_core.cpp -o test_core && ./test_core
#include <cstdio>
#include <cmath>
#include "SunSensorCore.h"

using namespace sun;
static int passed = 0, failed = 0;
#define CHECK(cond) do { if (cond) passed++; else { failed++; std::printf("FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
static bool near(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }

int main() {
  // Imbalance signs and magnitudes
  Imbalance c = computeImbalance(Quad{100, 100, 100, 100});
  CHECK(c.ok && near(c.ex, 0) && near(c.ey, 0));
  Imbalance r = computeImbalance(Quad{50, 150, 50, 150});  // right brighter
  CHECK(r.ok && near(r.ex, 0.5f) && near(r.ey, 0));
  Imbalance u = computeImbalance(Quad{150, 150, 50, 50});  // top brighter
  CHECK(u.ok && near(u.ex, 0) && near(u.ey, 0.5f));
  CHECK(!computeImbalance(Quad{0, 0, 0, 0}).ok);

  // Gain balancing makes mismatched channels equal
  Quad g;
  CHECK(balanceGains(Quad{90, 110, 100, 100}, g));
  Imbalance b = computeImbalance(applyGains(Quad{90, 110, 100, 100}, g));
  CHECK(near(b.ex, 0) && near(b.ey, 0));
  CHECK(!balanceGains(Quad{0, 110, 100, 100}, g));

  // Calibration table
  CalTable t;
  float d;
  CHECK(!t.ready());
  CHECK(t.add(-0.3f, -15) == CalResult::Added);
  CHECK(t.add(0.3f, 15) == CalResult::Added);
  CHECK(!t.ready());                        // no zero point yet
  CHECK(!t.lookup(0.0f, d));
  CHECK(t.add(0.0f, 0) == CalResult::Added);
  CHECK(t.ready());
  CHECK(t.lookup(0.15f, d) && near(d, 7.5f));
  CHECK(t.lookup(-0.3f, d) && near(d, -15));
  CHECK(!t.lookup(0.31f, d));               // no extrapolation
  CHECK(!t.lookup(NAN, d));
  CHECK(t.add(0.2f, 20) == CalResult::NotMonotonic);  // more angle, less imbalance
  CHECK(t.n == 3);                          // rejected point left table unchanged
  CHECK(t.add(0.301f, 16) == CalResult::TooClose);
  CHECK(t.add(0.02f, 0) == CalResult::Replaced);      // re-measure zero
  CHECK(t.n == 3 && t.lookup(0.02f, d) && near(d, 0));
  CHECK(t.add(0.5f, 95) == CalResult::BadInput);
  for (int i = 0; i < 4; i++) t.add(0.4f + 0.05f * i, 20.0f + i);
  CHECK(t.n == MAX_CAL_POINTS);
  CHECK(t.add(0.9f, 40) == CalResult::TableFull);

  // Separation angle
  CHECK(near(separationDeg(0, 0), 0));
  CHECK(near(separationDeg(20, 0), 20, 1e-3f));
  CHECK(near(separationDeg(10, 10), 14.03f, 0.05f));

  // Checksum detects a change
  unsigned char a1[4] = {1, 2, 3, 4}, a2[4] = {1, 2, 3, 5};
  CHECK(checksum(a1, 4) != checksum(a2, 4));

  std::printf("%d passed, %d failed\n", passed, failed);
  return failed ? 1 : 0;
}
