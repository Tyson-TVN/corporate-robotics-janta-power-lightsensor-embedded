// Config.h — hardware and tuning settings for the Janta Power Sun-offset sensor.
//
// Target: ESP32-DevKitC-32E (from the team Bill of Materials), 4x BPW34
// photodiodes, each through a transimpedance (current-to-voltage) op-amp stage
// powered at 3.3 V (MCP6004 per the FMEA).
//
// EVERYTHING BELOW IS A PROVISIONAL DEFAULT. None of these numbers come from a
// measured sensor head. Re-tune them after the first real light tests.
#pragma once

#ifndef ARDUINO_ARCH_ESP32
#error "This sketch is written for ESP32 (Tools > Board > esp32 > ESP32 Dev Module). Other boards need a review of ADC range, voltage and pins."
#endif

// ---------------------------------------------------------------------------
// Pins. Use ADC1 pins only (GPIO 32-39): ADC2 pins stop working when WiFi is
// on, and WiFi (NTP time, upload) is planned later.
// GPIO 34/35 are input-only, which is fine for analog sensing.
// Directions are defined LOOKING STRAIGHT AT THE SENSING FACE. Mark "UP" and
// "RIGHT" on the housing so everyone uses the same convention.
// ---------------------------------------------------------------------------
static const int PIN_TL = 32;  // top-left photodiode channel
static const int PIN_TR = 33;  // top-right
static const int PIN_BL = 34;  // bottom-left
static const int PIN_BR = 35;  // bottom-right

// ---------------------------------------------------------------------------
// ADC
// ---------------------------------------------------------------------------
static const int ADC_BITS = 12;         // ESP32 ADC: 0..4095
static const int ADC_MAX = 4095;
static const int OVERSAMPLE = 8;        // raw reads averaged per channel per sample

// Set true if brighter light gives a LOWER reading with your op-amp circuit
// (for example an inverting stage referenced to 3.3 V).
static const bool INVERT_POLARITY = false;

// ---------------------------------------------------------------------------
// Timing (milliseconds)
// ---------------------------------------------------------------------------
static const unsigned long SAMPLE_MS = 50;     // read sensors every 50 ms
static const unsigned long REPORT_MS = 250;    // print one CSV line every 250 ms
static const unsigned long WARMUP_MS = 500;    // ignore output right after start/mode change
static const unsigned long SETTLE_MS = 1000;   // readings must be steady this long

// ---------------------------------------------------------------------------
// Signal checks (in ADC counts, after oversampling)
// ---------------------------------------------------------------------------
// The ESP32 ADC is non-linear near the top of its range, so treat anything
// above ~3900 as saturated rather than waiting for 4095.
static const int SATURATION_COUNTS = 3900;
// If the SUM of all four channels is below this, there is not enough light.
static const long LOW_LIGHT_SUM = 400;
// A single channel at or below this while the others are clearly lit is
// treated as a broken wire / solder joint (FMEA: "channel reads 0").
static const int STUCK_LOW_COUNTS = 5;
static const long STUCK_CHECK_MIN_SUM = 1200;

// ---------------------------------------------------------------------------
// Filtering and stability
// ---------------------------------------------------------------------------
static const float SMOOTHING = 0.25f;          // exponential smoothing factor (0..1)
static const float STABLE_DELTA = 0.004f;      // max change in ex/ey per sample to count as steady
static const int CAL_AVERAGE_SAMPLES = 20;     // samples averaged when recording a calibration point

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
// "Near normal" = both axes within this many degrees. This is a display
// threshold only, NOT a measured accuracy or an approved acceptance criterion.
static const float NEAR_NORMAL_DEG = 2.0f;

static const unsigned long SERIAL_BAUD = 115200;
