# Janta Power – Sun-Offset Sensor (Embedded Software)

Corporate Robotics (Comet Robotics, UTD) × Janta Power, Embedded subteam.

This firmware covers the ClickUp task **"Software algorithm"**: *turn the sensor readings into an angle that shows how far the Sun is.*

> **Status: prototype v0.1, October 19 2026.**
> - The math is tested on a desktop: 30 core checks plus 24 checks that run the real sketch against fake hardware.
> - It has **not** been compiled for a real ESP32 yet; the toolchain wasn't available where it was written.
> - It has **not** been run on the real photodiode head.
> - All thresholds and the calibration flow are provisional.

## Hardware assumed (from the team BOM and FMEA)

| Item | Value |
| --- | --- |
| Board | ESP32-DevKitC-32E (3.3 V, 12-bit ADC: 0–4095) |
| Sensors | 4 × BPW34 photodiodes, each through a transimpedance op-amp stage (MCP6004 at 3.3 V) |
| Pins (default, change in `Config.h`) | TL = GPIO 32, TR = GPIO 33, BL = GPIO 34, BR = GPIO 35 |
| Optics | Tube that limits side light; quadrant layout of the four diodes |

- **Why these pins:** they are ADC1 pins, which keep working when WiFi is on (ADC2 pins don't).
- **Directions:** TL/TR/BL/BR are defined **looking straight at the sensing face**. Mark UP and RIGHT on the housing.
- **Op-amp output:** it must never exceed 3.3 V (FMEA, RPN 160). Check the feedback resistor against full sunlight.

## How it works

1. **Read.** Every 50 ms, read each channel. Each reading is the average of 8 ADC samples.
2. **Reject bad readings before filtering:**
   - `SATURATED`: any channel ≥ 3900
   - `LOW_LIGHT`: the four channels sum to less than 400
   - `CHANNEL_FAULT`: one channel ≈ 0 while the others are lit (broken wire or joint)
3. **Correct and smooth.** Apply per-channel gains (set by `balance`), then exponential smoothing.
4. **Compute imbalance.** With S = TL + TR + BL + BR:
   - `ex = (TR + BR − TL − BL) / S`, positive when the right side is brighter
   - `ey = (TL + TR − BL − BR) / S`, positive when the top is brighter

   These are **not angles yet**.
5. **Wait until steady.** `ex` and `ey` must stay steady for 1 s (`stable`).
6. **Convert to degrees with measured calibration tables**, one per axis:
   - Each table holds up to 7 points and needs a negative, a zero and a positive point.
   - Values between points are interpolated linearly. Values outside the measured range give `OUT_OF_RANGE`; the code never extrapolates.
   - There is deliberately **no** fixed formula like "imbalance × 90". The real response depends on the tube and the diodes, so it has to be measured.
7. **Report the angles:**
   - `x_deg`: Sun offset toward sensor-RIGHT
   - `y_deg`: Sun offset toward sensor-UP
   - `separation_deg = atan(√(tan²x + tan²y))`: total angle from the panel normal. This assumes the two axes respond independently; check it with combined x+y tests.

## Run it (Arduino IDE)

1. **Add the ESP32 board package.** In *File → Preferences → Additional boards manager URLs*, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
   Then go to *Boards Manager*, search "esp32" (by Espressif), and install it.
2. **Open the sketch:** `firmware/JantaPower_SunSensor/JantaPower_SunSensor.ino`. All three files in that folder must stay together.
3. **Select the board:** *Tools → Board → esp32 → ESP32 Dev Module*, then pick the port. Click **Verify**, then **Upload**.
4. **Open Serial Monitor** at **115200 baud**, line ending **Newline**.

It starts in **DEMO** mode with synthetic readings, so nothing needs to be wired. Type `help`.

| Command | What it does |
| --- | --- |
| `demo 0`…`demo 6` | Synthetic data: center, Sun 20° right, up-left (−10°, +10°), darkness, saturation, outside calibrated range, broken channel |
| `live` | Read the real photodiodes, using the LIVE calibration |
| `balance` | LIVE only. Equalize channel gains. Use only when the head is independently known to point straight at the Sun. Clears the angle tables. |
| `cal x DEG` / `cal y DEG` | LIVE only. Record the current steady reading as a known angle on one axis (hold the other axis at 0). |
| `show` | Print the gains and calibration tables |
| `save` / `load` | Store or restore the LIVE calibration in ESP32 flash (versioned and checksummed) |
| `clear` | Reset the LIVE calibration and erase the saved copy |
| `header` | Reprint the CSV column names |

DEMO and LIVE calibrations are separate, so synthetic data can never become the real calibration.

### Output (one CSV line every 250 ms)

```
t_ms,mode,status,tl,tr,bl,br,ex,ey,x_deg,y_deg,separation_deg,signal_usable,angle_valid,stable
```

- The `tl`…`br` columns are raw ADC counts.
- Invalid values print as `NA`, never `0`.
- Lines starting with `#` are messages for humans.
- **Status values:** `WARMUP`, `LOW_LIGHT`, `SATURATED`, `CHANNEL_FAULT`, `UNCALIBRATED`, `SETTLING`, `OUT_OF_RANGE`, `ANGLE_ESTIMATE`, `NEAR_NORMAL_ESTIMATE` (both axes within ±2°; a display threshold, not a measured accuracy).

Example from the desktop run, `demo 1`:

```
4051,DEMO,ANGLE_ESTIMATE,706,1689,706,1689,0.4104,0.0000,20.17,0.00,20.17,1,1,1
```

The 0.17° error comes from linear interpolation between the 15° and 30° demo calibration points. More calibration points shrink it.

## First real-sensor calibration (proposed procedure)

1. Wire the head, type `live`, and check that each channel rises when you shine light on it. This confirms the pins and the polarity.
2. Outdoors in steady direct sun, point the head straight at the Sun using an independent reference. Wait for `stable=1`, then type `balance`.
3. Tilt **only the x axis** to known angles, for example −20, −10, 0, +10, +20. At each one, wait for `stable=1`, then type `cal x <angle>`. Repeat for y.
   - **Sign rule:** to make the Sun appear +20° to the right, point the head 20° to the *left* of the Sun.
4. `save`. Then check angles you did **not** calibrate, including combined x+y angles, and record the errors. That is the test-report data.

## Tests (desktop, no board needed)

```
cd tests
g++ -std=c++17 -I../firmware/JantaPower_SunSensor test_core.cpp -o test_core && ./test_core
g++ -std=c++17 -Imock -I../firmware/JantaPower_SunSensor -x c++ test_sketch.cpp -o test_sketch && ./test_sketch
```

- `test_core`: the math. Imbalance, gain balancing, calibration rules, no extrapolation, separation angle, checksum.
- `test_sketch`: runs the actual `.ino` with fake ADC, Serial, flash and clock. It covers all DEMO scenarios, the LIVE balance-and-calibrate flow, save/load, corrupted saved data and `millis()` wrap-around.
- `docs/DEMO_OUTPUT.txt`: a transcript from that run. It is synthetic, not a board capture.

## Not done yet

- Real ESP32 compile and upload
- Real sensor characterization and calibration
- Outdoor accuracy and repeatability
- Reflection/outlier rejection (FMEA RPN 150)
- Temperature logging

The wider system in the FMEA is also **not implemented**:
- NTP time
- Latitude/longitude and the Sun-position calculation (heading)
- Home/limit-switch confirmation
- Tower ID confirmation
- Offline storage and AWS S3 upload
