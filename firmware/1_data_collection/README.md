# Firmware 1 — Data Collection

## Purpose

This firmware runs the **ring-down measurement experiment**. It:

1. Moves the robot slowly to a test start position (no logging)
2. For each of 3 trials:
   - Records accelerometer data **before** the move (phase 0, 700 ms)
   - Executes the X-axis move **with ZVD input shaping** (phase 1)
   - Records accelerometer data **after** the move (phase 2, 2200 ms)
3. Dumps all data to Serial as CSV
4. Returns to (0, 0)

## What to configure before flashing

At the top of `data_collection.ino`, change these constants to match your test case:

```cpp
// The test position and target
const float TEST_START_X_CM  = 22.2f;  // Start X (cm)
const float TEST_START_Y_CM  = 4.5f;   // Y row being tested
const float TEST_TARGET_X_CM = 17.2f;  // End X (cm)

// The shaper to deploy (update after running identify_fn_zeta.m)
const float SHAPER_FN   = 11.0f;   // Design frequency (Hz)
const float SHAPER_ZETA = 0.030f;  // Damping ratio
// A1, A2, A3, T1, T2, T3 are computed from the above
```

To run **without shaping** (for baseline), you can either comment out the shaper convolution in `moveXMeasuredShapedZVD()` and call `baseDistanceCm()` directly, or use a separate no-shaper version of the move function.

## Wiring

| ESP32 Pin | Connected to |
|---|---|
| GPIO 26 | X-axis STEP |
| GPIO 27 | X-axis DIR |
| GPIO 25 | Y-axis STEP |
| GPIO 33 | Y-axis DIR |
| GPIO 21 | MPU6050 SDA |
| GPIO 22 | MPU6050 SCL |

MPU6050 VCC → 3.3V, GND → GND, AD0 → GND (I2C address 0x68)

## Data output

Data is printed to Serial at 115200 baud in CSV format:

```
trial,t_us,phase,x_cm,y_cm,ax,ay,az
1,0,0,22200,4500,312,-45,16384
...
```

- `phase`: 0 = before move, 1 = during move, 2 = after move (ring-down)
- `ax`, `ay`, `az`: raw MPU6050 ADC counts (±2g range → divide by 16384 for g, multiply by 9.81 for m/s²)

Copy the Serial output to a `.csv` file and place it in `data/raw_csv/`.

## How to flash (PlatformIO)

```bash
pio run --target upload --environment esp32dev
```

Or use Arduino IDE with the ESP32 board package installed (select "ESP32 Dev Module").
