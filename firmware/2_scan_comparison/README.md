# Firmware 2 — AOI Scan Comparison

## Purpose

This firmware runs the **full PCB scan route** twice in sequence:

1. **Run 1 — Without input shaping:** All scan moves use plain trapezoidal profiles
2. **Run 2 — With ZVD input shaping:** X-axis moves use per-row shapers; Y-axis transitions use per-segment shapers

This allows a direct side-by-side comparison of the robot's behavior with and without vibration suppression over a realistic AOI scan pattern.

## Scan Route

```
Start at (0,0) → slow to reference (24.7, 17.0) → slow to first scan point (22.2, 14.5)

Row y=14.5:  (22.2,14.5) → (17.2,14.5) → (12.2,14.5) → (7.2,14.5)
             ↓ transition to y=9.5
Row y=9.5:   (7.2, 9.5)  → (12.2, 9.5) → (17.2, 9.5) → (22.2, 9.5)
             ↓ transition to y=4.5
Row y=4.5:   (22.2, 4.5) → (17.2, 4.5) → (12.2, 4.5) → (7.2, 4.5)

Return slow to (0,0)
```

## Shaper Lookup Table

The correct X-axis shaper is selected automatically based on the current Y row:

| Y row (cm) | fₙ deployed (Hz) | ζ | A₁ | A₂ | A₃ | T₂ (s) | T₃ (s) |
|---|---|---|---|---|---|---|---|
| 14.5 | 11.1 | 0.035 | 0.2782 | 0.4985 | 0.2233 | 0.04507 | 0.09015 |
| 9.5  | 11.3 | 0.035 | 0.2782 | 0.4985 | 0.2233 | 0.04428 | 0.08855 |
| 4.5  | 11.6 | 0.035 | 0.2782 | 0.4985 | 0.2233 | 0.04310 | 0.08621 |

Y-axis transitions also use dedicated shapers tuned for that movement direction.

## Key Configuration Constants

```cpp
// Motion profile (scan moves)
const float FAST_V_CM_S  = 30.0f;    // Max velocity (cm/s)
const float FAST_A_CM_S2 = 220.0f;   // Acceleration (cm/s²)

// Steps per cm (depends on your belt/pulley ratio)
// GT2 belt, 20-tooth pulley, 1/8 microstepping, 200 steps/rev:
// steps/rev * microstep / (pulley_teeth * belt_pitch) = 200*8/(20*0.2) = 400
const float X_STEPS_PER_CM = 400.0f;
const float Y_STEPS_PER_CM = 400.0f;
```

## How to update shaper coefficients

If you re-identify the system (new Z position, reconfigured robot), run `matlab/identify_fn_zeta.m` to get the new fₙ and ζ, then recompute the ZVD coefficients using:

```
K = exp(-ζ * π / sqrt(1 - ζ²))
A1 = 1 / (1 + 2K + K²)
A2 = 2K / (1 + 2K + K²)
A3 = K² / (1 + 2K + K²)
ωd = 2π * fₙ * sqrt(1 - ζ²)
T2 = π / ωd
T3 = 2π / ωd
```

Or just run the MATLAB script `matlab/zvd_simulation.m` which prints the coefficients automatically.

## Wiring

Same as Firmware 1 — but **no MPU6050 required** for this scan comparison firmware (it only controls motors).

| ESP32 Pin | Connected to |
|---|---|
| GPIO 26 | X-axis STEP |
| GPIO 27 | X-axis DIR |
| GPIO 25 | Y-axis STEP |
| GPIO 33 | Y-axis DIR |
