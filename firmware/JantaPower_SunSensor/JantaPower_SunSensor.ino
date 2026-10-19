// JantaPower_SunSensor.ino
// Corporate Robotics x Janta Power — Embedded / Software
// ClickUp task "Software algorithm": turn the sensor readings into an angle
// that shows how far the Sun is from where the panel points.
//
// Board: ESP32-DevKitC-32E.  Open Serial Monitor at 115200 baud, "Newline".
// Starts in DEMO mode (synthetic readings, no wiring needed). Type "help".
//
// PROTOTYPE STATUS: compiles and runs the math on synthetic data. It has NOT
// been tested with the real photodiode head; thresholds and calibration are
// provisional. See README.md.

#include <Preferences.h>
#include "Config.h"
#include "SunSensorCore.h"

using sun::Quad;
using sun::CalTable;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
enum class Mode { DEMO, LIVE };
enum class Status {
  WARMUP, LOW_LIGHT, SATURATED, CHANNEL_FAULT, UNCALIBRATED,
  SETTLING, OUT_OF_RANGE, ANGLE_ESTIMATE, NEAR_NORMAL_ESTIMATE
};

const char* statusText(Status s) {
  switch (s) {
    case Status::WARMUP: return "WARMUP";
    case Status::LOW_LIGHT: return "LOW_LIGHT";
    case Status::SATURATED: return "SATURATED";
    case Status::CHANNEL_FAULT: return "CHANNEL_FAULT";
    case Status::UNCALIBRATED: return "UNCALIBRATED";
    case Status::SETTLING: return "SETTLING";
    case Status::OUT_OF_RANGE: return "OUT_OF_RANGE";
    case Status::ANGLE_ESTIMATE: return "ANGLE_ESTIMATE";
    default: return "NEAR_NORMAL_ESTIMATE";
  }
}

// Calibration for one mode. DEMO and LIVE are kept separate so synthetic data
// can never end up in the real calibration.
struct Calibration {
  Quad gain{1, 1, 1, 1};
  CalTable x, y;
};

Calibration liveCal;
Calibration demoCal;

Mode mode = Mode::DEMO;
int demoScenario = 0;

// Latest measurement
Quad raw{0, 0, 0, 0};        // ADC counts before polarity/gain/filter
Quad filtered{0, 0, 0, 0};   // gain-corrected and smoothed
bool haveFiltered = false;
float ex = NAN, ey = NAN;
float xDeg = NAN, yDeg = NAN, sepDeg = NAN;
bool signalUsable = false, angleValid = false, stable = false;
Status status = Status::WARMUP;

unsigned long modeStartMs = 0;
unsigned long lastSampleMs = 0, lastReportMs = 0;
unsigned long stableSinceMs = 0;
bool stableTracking = false;
float prevEx = NAN, prevEy = NAN;

Preferences prefs;

Calibration& activeCal() { return mode == Mode::LIVE ? liveCal : demoCal; }

// ---------------------------------------------------------------------------
// DEMO data
// A simple synthetic sensor: imbalance = 1.2 * sin(angle). Only for showing
// the pipeline — the real head's response will be different.
// ---------------------------------------------------------------------------
const float DEMO_K = 1.2f;

Quad demoQuad(float xd, float yd, float base) {
  const float D2R = 3.14159265f / 180.0f;
  float sx = DEMO_K * sinf(xd * D2R), sy = DEMO_K * sinf(yd * D2R);
  // tiny deterministic flicker so the numbers look alive
  float n = 1.0f + 0.002f * sinf(millis() / 137.0f);
  Quad q{base * (1 - sx + sy) * n, base * (1 + sx + sy) * n,
         base * (1 - sx - sy) * n, base * (1 + sx - sy) * n};
  if (q.tl < 0) q.tl = 0;
  if (q.tr < 0) q.tr = 0;
  if (q.bl < 0) q.bl = 0;
  if (q.br < 0) q.br = 0;
  return q;
}

void buildDemoCalibration() {
  demoCal = Calibration();
  const float pts[] = {-30, -15, 0, 15, 30};
  const float D2R = 3.14159265f / 180.0f;
  for (float d : pts) {
    demoCal.x.add(DEMO_K * sinf(d * D2R), d);
    demoCal.y.add(DEMO_K * sinf(d * D2R), d);
  }
}

Quad readDemo() {
  switch (demoScenario) {
    case 0: return demoQuad(0, 0, 1200);        // centered
    case 1: return demoQuad(20, 0, 1200);       // Sun 20 deg to the right
    case 2: return demoQuad(-10, 10, 1200);     // Sun up-left
    case 3: return demoQuad(0, 0, 20);          // darkness
    case 4: return demoQuad(0, 0, 4000);        // saturated
    case 5: return demoQuad(45, 0, 1200);       // outside calibrated range
    default: {                                   // 6: broken BL wire
      Quad q = demoQuad(0, 0, 1200);
      q.bl = 0;
      return q;
    }
  }
}

// ---------------------------------------------------------------------------
// LIVE data
// ---------------------------------------------------------------------------
float readChannel(int pin) {
  long sum = 0;
  for (int i = 0; i < OVERSAMPLE; i++) sum += analogRead(pin);
  float v = (float)sum / OVERSAMPLE;
  return INVERT_POLARITY ? (ADC_MAX - v) : v;
}

Quad readLive() {
  return Quad{readChannel(PIN_TL), readChannel(PIN_TR), readChannel(PIN_BL), readChannel(PIN_BR)};
}

// ---------------------------------------------------------------------------
// Measurement pipeline (runs every SAMPLE_MS)
// ---------------------------------------------------------------------------
void resetFilter() {
  haveFiltered = false;
  stableTracking = false;
  stable = false;
  prevEx = prevEy = NAN;
}

void sample(unsigned long now) {
  raw = (mode == Mode::LIVE) ? readLive() : readDemo();
  Calibration& cal = activeCal();

  xDeg = yDeg = sepDeg = NAN;
  angleValid = false;
  signalUsable = false;

  // 1) Checks on the UNFILTERED readings, so one bad sample is caught at once.
  float mx = max(max(raw.tl, raw.tr), max(raw.bl, raw.br));
  float mn = min(min(raw.tl, raw.tr), min(raw.bl, raw.br));
  float sum = raw.tl + raw.tr + raw.bl + raw.br;

  if (now - modeStartMs < WARMUP_MS) { status = Status::WARMUP; resetFilter(); return; }
  if (mx >= SATURATION_COUNTS) { status = Status::SATURATED; resetFilter(); return; }
  if (sum < LOW_LIGHT_SUM) { status = Status::LOW_LIGHT; resetFilter(); return; }
  if (sum >= STUCK_CHECK_MIN_SUM && mn <= STUCK_LOW_COUNTS) {
    status = Status::CHANNEL_FAULT; resetFilter(); return;
  }

  // 2) Gain correction + exponential smoothing.
  Quad g = sun::applyGains(raw, cal.gain);
  if (!haveFiltered) { filtered = g; haveFiltered = true; }
  else {
    filtered.tl += SMOOTHING * (g.tl - filtered.tl);
    filtered.tr += SMOOTHING * (g.tr - filtered.tr);
    filtered.bl += SMOOTHING * (g.bl - filtered.bl);
    filtered.br += SMOOTHING * (g.br - filtered.br);
  }

  // 3) Imbalance.
  sun::Imbalance im = sun::computeImbalance(filtered);
  if (!im.ok) { status = Status::LOW_LIGHT; resetFilter(); return; }
  ex = im.ex; ey = im.ey;
  signalUsable = true;

  // 4) Stability: ex/ey must stop moving for SETTLE_MS.
  bool steadyNow = !isnan(prevEx) && fabsf(ex - prevEx) < STABLE_DELTA && fabsf(ey - prevEy) < STABLE_DELTA;
  prevEx = ex; prevEy = ey;
  if (steadyNow) {
    if (!stableTracking) { stableTracking = true; stableSinceMs = now; }
  } else {
    stableTracking = false;
  }
  stable = stableTracking && (now - stableSinceMs >= SETTLE_MS);

  // 5) Angles from measured calibration.
  if (!cal.x.ready() || !cal.y.ready()) { status = Status::UNCALIBRATED; return; }
  if (!stable) { status = Status::SETTLING; return; }
  float xd, yd;
  if (!cal.x.lookup(ex, xd) || !cal.y.lookup(ey, yd)) { status = Status::OUT_OF_RANGE; return; }
  xDeg = xd; yDeg = yd;
  sepDeg = sun::separationDeg(xd, yd);
  angleValid = true;
  status = (fabsf(xd) <= NEAR_NORMAL_DEG && fabsf(yd) <= NEAR_NORMAL_DEG)
               ? Status::NEAR_NORMAL_ESTIMATE : Status::ANGLE_ESTIMATE;
}

// ---------------------------------------------------------------------------
// Output: one CSV line. Invalid values are "NA", never 0.
// ---------------------------------------------------------------------------
void printNum(float v, int digits) {
  if (isnan(v)) Serial.print("NA");
  else Serial.print(v, digits);
}

void printHeader() {
  Serial.println("t_ms,mode,status,tl,tr,bl,br,ex,ey,x_deg,y_deg,separation_deg,signal_usable,angle_valid,stable");
}

void report(unsigned long now) {
  Serial.print(now); Serial.print(',');
  Serial.print(mode == Mode::LIVE ? "LIVE" : "DEMO"); Serial.print(',');
  Serial.print(statusText(status)); Serial.print(',');
  Serial.print(raw.tl, 0); Serial.print(','); Serial.print(raw.tr, 0); Serial.print(',');
  Serial.print(raw.bl, 0); Serial.print(','); Serial.print(raw.br, 0); Serial.print(',');
  bool showImb = signalUsable;
  printNum(showImb ? ex : NAN, 4); Serial.print(',');
  printNum(showImb ? ey : NAN, 4); Serial.print(',');
  printNum(xDeg, 2); Serial.print(','); printNum(yDeg, 2); Serial.print(',');
  printNum(sepDeg, 2); Serial.print(',');
  Serial.print(signalUsable ? 1 : 0); Serial.print(',');
  Serial.print(angleValid ? 1 : 0); Serial.print(',');
  Serial.println(stable ? 1 : 0);
}

// ---------------------------------------------------------------------------
// Save / load LIVE calibration in ESP32 flash (Preferences / NVS)
// ---------------------------------------------------------------------------
const uint32_t CAL_VERSION = 1;
struct SavedCal { uint32_t version; Calibration cal; uint32_t check; };

bool saveCal() {
  SavedCal s;
  s.version = CAL_VERSION;
  s.cal = liveCal;
  s.check = sun::checksum((const unsigned char*)&s, offsetof(SavedCal, check));
  prefs.begin("janta", false);
  size_t n = prefs.putBytes("cal", &s, sizeof(s));
  prefs.end();
  return n == sizeof(s);
}

bool loadCal() {
  SavedCal s;
  prefs.begin("janta", true);
  size_t n = prefs.getBytes("cal", &s, sizeof(s));
  prefs.end();
  if (n != sizeof(s) || s.version != CAL_VERSION) return false;
  if (s.check != sun::checksum((const unsigned char*)&s, offsetof(SavedCal, check))) return false;
  liveCal = s.cal;
  return true;
}

void eraseSavedCal() {
  prefs.begin("janta", false);
  prefs.remove("cal");
  prefs.end();
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
void printHelp() {
  Serial.println(F("# Commands:"));
  Serial.println(F("#  help              this list"));
  Serial.println(F("#  demo N            synthetic data: 0 center, 1 right 20deg, 2 up-left, 3 dark,"));
  Serial.println(F("#                    4 saturated, 5 out of range, 6 broken channel"));
  Serial.println(F("#  live              read the real photodiodes (uses LIVE calibration)"));
  Serial.println(F("#  balance           LIVE: equalize channel gains. Only when the head points"));
  Serial.println(F("#                    straight at the Sun. Clears angle tables."));
  Serial.println(F("#  cal x DEG         LIVE: record current reading as a known x angle (e.g. cal x -10)"));
  Serial.println(F("#  cal y DEG         LIVE: same for y. Need negative, 0 and positive points per axis."));
  Serial.println(F("#  show              print calibration in use"));
  Serial.println(F("#  save / load       store / restore LIVE calibration in flash"));
  Serial.println(F("#  clear             reset LIVE calibration and erase the saved copy"));
  Serial.println(F("#  header            print the CSV column names again"));
}

void setMode(Mode m) {
  mode = m;
  modeStartMs = millis();
  resetFilter();
  Serial.print(F("# mode ")); Serial.println(m == Mode::LIVE ? "LIVE" : "DEMO");
}

void showCal() {
  Calibration& c = activeCal();
  Serial.print(F("# calibration for ")); Serial.println(mode == Mode::LIVE ? "LIVE" : "DEMO");
  Serial.print(F("#  gains tl,tr,bl,br: "));
  Serial.print(c.gain.tl, 4); Serial.print(' '); Serial.print(c.gain.tr, 4); Serial.print(' ');
  Serial.print(c.gain.bl, 4); Serial.print(' '); Serial.println(c.gain.br, 4);
  const CalTable* t[2] = {&c.x, &c.y};
  for (int a = 0; a < 2; a++) {
    Serial.print(a == 0 ? F("#  x table") : F("#  y table"));
    Serial.print(t[a]->ready() ? F(" (ready): ") : F(" (NOT ready): "));
    for (int i = 0; i < t[a]->n; i++) {
      Serial.print(t[a]->deg[i], 1); Serial.print("deg@"); Serial.print(t[a]->imb[i], 4); Serial.print("  ");
    }
    Serial.println();
  }
}

// Average several steady samples for a calibration point.
bool averagedImbalance(float& axOut, float& ayOut) {
  float sx = 0, sy = 0;
  int got = 0;
  for (int i = 0; i < CAL_AVERAGE_SAMPLES; i++) {
    delay(SAMPLE_MS);
    sample(millis());
    if (!signalUsable) return false;
    sx += ex; sy += ey; got++;
  }
  axOut = sx / got; ayOut = sy / got;
  return true;
}

void handleCommand(char* line) {
  // lower-case and trim
  for (char* p = line; *p; p++) *p = tolower(*p);
  while (*line == ' ') line++;
  int len = strlen(line);
  while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\r')) line[--len] = 0;
  if (len == 0) return;

  if (!strcmp(line, "help")) { printHelp(); return; }
  if (!strcmp(line, "header")) { printHeader(); return; }
  if (!strcmp(line, "show")) { showCal(); return; }
  if (!strcmp(line, "live")) { setMode(Mode::LIVE); return; }
  if (!strncmp(line, "demo", 4)) {
    int n = 0;
    if (sscanf(line + 4, "%d", &n) == 1 && n >= 0 && n <= 6) demoScenario = n;
    setMode(Mode::DEMO);
    Serial.print(F("# demo scenario ")); Serial.println(demoScenario);
    return;
  }

  bool isLiveCmd = !strcmp(line, "balance") || !strncmp(line, "cal ", 4) ||
                   !strcmp(line, "save") || !strcmp(line, "load") || !strcmp(line, "clear");
  if (isLiveCmd && mode != Mode::LIVE) {
    Serial.println(F("# calibration commands only work in LIVE mode (type: live)"));
    return;
  }

  if (!strcmp(line, "clear")) {
    liveCal = Calibration(); eraseSavedCal(); resetFilter();
    Serial.println(F("# LIVE calibration cleared")); return;
  }
  if (!strcmp(line, "save")) { Serial.println(saveCal() ? F("# saved") : F("# save FAILED")); return; }
  if (!strcmp(line, "load")) {
    if (loadCal()) { resetFilter(); Serial.println(F("# loaded")); }
    else Serial.println(F("# no valid saved calibration"));
    return;
  }
  if (!strcmp(line, "balance")) {
    if (!signalUsable) { Serial.println(F("# balance refused: signal not usable")); return; }
    Quad g;
    if (!sun::balanceGains(raw, g)) { Serial.println(F("# balance refused: a channel reads zero")); return; }
    liveCal.gain = g; liveCal.x.clear(); liveCal.y.clear(); resetFilter();
    Serial.println(F("# gains balanced; angle tables cleared (re-run cal x / cal y)"));
    return;
  }
  if (!strncmp(line, "cal ", 4)) {
    char axis = 0; float d = 0;
    if (sscanf(line + 4, " %c %f", &axis, &d) != 2 || (axis != 'x' && axis != 'y')) {
      Serial.println(F("# usage: cal x DEG   or   cal y DEG")); return;
    }
    if (!stable) { Serial.println(F("# cal refused: wait until stable=1")); return; }
    float ax, ay;
    if (!averagedImbalance(ax, ay)) { Serial.println(F("# cal refused: signal dropped while averaging")); return; }
    CalTable& t = (axis == 'x') ? liveCal.x : liveCal.y;
    sun::CalResult r = t.add(axis == 'x' ? ax : ay, d);
    Serial.print(F("# cal ")); Serial.print(axis); Serial.print(' '); Serial.print(d, 1);
    Serial.print(F(" -> ")); Serial.println(sun::calResultText(r));
    return;
  }
  Serial.println(F("# unknown command (type help)"));
}

char cmdBuf[48];
int cmdLen = 0;
bool cmdOverflow = false;

void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      if (cmdOverflow) Serial.println(F("# command too long"));
      else handleCommand(cmdBuf);
      cmdLen = 0; cmdOverflow = false;
    } else if (cmdLen < (int)sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;
    }
  }
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  analogReadResolution(ADC_BITS);
  analogSetAttenuation(ADC_11db);  // ~0..3.1 V usable input range
  pinMode(PIN_TL, INPUT); pinMode(PIN_TR, INPUT);
  pinMode(PIN_BL, INPUT); pinMode(PIN_BR, INPUT);

  buildDemoCalibration();
  bool loaded = loadCal();

  Serial.println(F("# Janta Power Sun-offset sensor - prototype v0.1 (ESP32)"));
  Serial.println(F("# Starting in DEMO mode with synthetic data. Type help."));
  Serial.println(loaded ? F("# saved LIVE calibration loaded") : F("# no saved LIVE calibration"));
  setMode(Mode::DEMO);
  printHeader();
}

void loop() {
  pollSerial();
  unsigned long now = millis();
  // Unsigned subtraction keeps working when millis() wraps around.
  if (now - lastSampleMs >= SAMPLE_MS) { lastSampleMs = now; sample(now); }
  if (now - lastReportMs >= REPORT_MS) { lastReportMs = now; report(now); }
}
