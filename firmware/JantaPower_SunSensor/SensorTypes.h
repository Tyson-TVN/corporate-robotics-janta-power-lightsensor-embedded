// SensorTypes.h — types used by the main sketch.
//
// Why this file exists: before compiling a .ino, Arduino IDE automatically
// adds a declaration for every function near the top of the file. If a
// function returns or takes a custom type (like Calibration) that is defined
// later in the .ino, those auto-added lines fail with "does not name a type".
// Defining the types in a header that is #included first avoids that.
#pragma once
#include "SunSensorCore.h"

using sun::Quad;
using sun::CalTable;

enum class Mode { DEMO, LIVE };

enum class Status {
  WARMUP, LOW_LIGHT, SATURATED, CHANNEL_FAULT, UNCALIBRATED,
  SETTLING, OUT_OF_RANGE, ANGLE_ESTIMATE, NEAR_NORMAL_ESTIMATE
};

// Calibration for one mode. DEMO and LIVE are kept separate so synthetic data
// can never end up in the real calibration.
struct Calibration {
  Quad gain{1, 1, 1, 1};
  CalTable x, y;
};
