// SunSensorCore.h — the math, with no Arduino dependencies.
//
// Because nothing here touches hardware, the same file is compiled by the
// desktop unit tests in /tests. If you change the math, rerun those tests.
//
// Pipeline:
//   4 brightness values -> per-channel gain correction -> imbalance (ex, ey)
//   -> measured calibration table per axis -> angles in degrees.
#pragma once
#include <math.h>

namespace sun {

// Four channels, named by position looking at the sensing face.
struct Quad {
  float tl, tr, bl, br;
};

// Dimensionless imbalance. NOT an angle.
//   ex > 0 : right-side sensors brighter than left
//   ey > 0 : top sensors brighter than bottom
struct Imbalance {
  float ex, ey;
  bool ok;  // false if the total was zero / not usable
};

inline Quad applyGains(const Quad& raw, const Quad& gain) {
  return Quad{raw.tl * gain.tl, raw.tr * gain.tr, raw.bl * gain.bl, raw.br * gain.br};
}

inline Imbalance computeImbalance(const Quad& q) {
  float s = q.tl + q.tr + q.bl + q.br;
  if (!(s > 0.0f) || isnan(s) || isinf(s)) return Imbalance{0, 0, false};
  return Imbalance{(q.tr + q.br - q.tl - q.bl) / s,
                   (q.tl + q.tr - q.bl - q.br) / s, true};
}

// Gains that make all four channels read equal under the CURRENT light.
// Only use this when the head is independently known to be pointing straight
// at the Sun (or at a uniform source) — otherwise you bake an error in.
inline bool balanceGains(const Quad& raw, Quad& gainOut) {
  if (raw.tl <= 0 || raw.tr <= 0 || raw.bl <= 0 || raw.br <= 0) return false;
  float mean = (raw.tl + raw.tr + raw.bl + raw.br) / 4.0f;
  gainOut = Quad{mean / raw.tl, mean / raw.tr, mean / raw.bl, mean / raw.br};
  return true;
}

// ---------------------------------------------------------------------------
// Calibration table: measured (imbalance -> degrees) points for one axis.
// We do NOT assume a formula like "angle = imbalance * 90". The real response
// depends on the tube geometry and the diodes, so it has to be measured.
// ---------------------------------------------------------------------------
static const int MAX_CAL_POINTS = 7;
static const float MIN_IMBALANCE_SPACING = 0.002f;  // points must be distinguishable

enum class CalResult { Added, Replaced, TableFull, NotMonotonic, TooClose, BadInput };

inline const char* calResultText(CalResult r) {
  switch (r) {
    case CalResult::Added: return "added";
    case CalResult::Replaced: return "replaced";
    case CalResult::TableFull: return "table full (max 7 points)";
    case CalResult::NotMonotonic: return "rejected: response not monotonic (more angle must give more imbalance)";
    case CalResult::TooClose: return "rejected: imbalance too close to an existing point";
    default: return "rejected: bad input";
  }
}

struct CalTable {
  int n = 0;
  float imb[MAX_CAL_POINTS];  // sorted ascending by deg
  float deg[MAX_CAL_POINTS];

  void clear() { n = 0; }

  // Usable only with a negative, a zero-ish and a positive point.
  bool ready() const {
    if (n < 3) return false;
    bool neg = false, pos = false, zero = false;
    for (int i = 0; i < n; i++) {
      if (deg[i] < -0.5f) neg = true;
      else if (deg[i] > 0.5f) pos = true;
      else zero = true;
    }
    return neg && pos && zero;
  }

  // The new table must stay strictly increasing in imbalance as degrees
  // increase. If the new point breaks that, the old table is left untouched.
  CalResult add(float imbalance, float degrees) {
    if (isnan(imbalance) || isnan(degrees) || fabsf(degrees) > 89.0f) return CalResult::BadInput;
    float ti[MAX_CAL_POINTS + 1], td[MAX_CAL_POINTS + 1];
    int m = 0;
    bool replaced = false;
    for (int i = 0; i < n; i++) {
      if (fabsf(deg[i] - degrees) < 0.01f) { replaced = true; continue; }  // re-measure same angle
      ti[m] = imb[i]; td[m] = deg[i]; m++;
    }
    if (m >= MAX_CAL_POINTS) return CalResult::TableFull;
    // insert sorted by degrees
    int pos = m;
    while (pos > 0 && td[pos - 1] > degrees) { ti[pos] = ti[pos - 1]; td[pos] = td[pos - 1]; pos--; }
    ti[pos] = imbalance; td[pos] = degrees; m++;
    for (int i = 1; i < m; i++) {
      float d = ti[i] - ti[i - 1];
      if (d <= 0) return CalResult::NotMonotonic;
      if (d < MIN_IMBALANCE_SPACING) return CalResult::TooClose;
    }
    for (int i = 0; i < m; i++) { imb[i] = ti[i]; deg[i] = td[i]; }
    n = m;
    return replaced ? CalResult::Replaced : CalResult::Added;
  }

  // Linear interpolation between measured points. Returns false outside the
  // measured range: we never extrapolate.
  bool lookup(float imbalance, float& degOut) const {
    if (!ready() || isnan(imbalance)) return false;
    if (imbalance < imb[0] || imbalance > imb[n - 1]) return false;
    for (int i = 1; i < n; i++) {
      if (imbalance <= imb[i]) {
        float t = (imbalance - imb[i - 1]) / (imb[i] - imb[i - 1]);
        degOut = deg[i - 1] + t * (deg[i] - deg[i - 1]);
        return true;
      }
    }
    return false;
  }
};

// Total angle between the Sun and the panel normal, from the two projected
// angles. Assumes the axes respond independently; must be checked with
// combined x+y tests on the real head.
inline float separationDeg(float xDeg, float yDeg) {
  const float D2R = 3.14159265f / 180.0f;
  float tx = tanf(xDeg * D2R), ty = tanf(yDeg * D2R);
  return atanf(sqrtf(tx * tx + ty * ty)) / D2R;
}

// Simple checksum for saved calibration data.
inline unsigned long checksum(const unsigned char* p, unsigned int len) {
  unsigned long h = 2166136261UL;  // FNV-1a
  for (unsigned int i = 0; i < len; i++) { h ^= p[i]; h *= 16777619UL; h &= 0xFFFFFFFFUL; }
  return h;
}

}  // namespace sun
