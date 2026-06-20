# How to Use This Project

This guide is written for **future UPTP capstone students** who want to reproduce the experiments, continue this research, or adapt it to a new robot configuration.

---

## Prerequisites

### Software you need to install

| Tool | Purpose | Download |
|------|---------|---------|
| Arduino IDE or PlatformIO | Flash ESP32 firmware | https://www.arduino.cc or https://platformio.org |
| ESP32 board package | Adds ESP32 support to Arduino IDE | Boards Manager → search "esp32" by Espressif |
| MATLAB R2021b or newer | Run system ID and simulation scripts | NTUST has a campus license |
| Python 3.9+ with pip | Process CSV data and generate figures | https://www.python.org |

Install Python dependencies:
```bash
pip install numpy pandas scipy matplotlib
```

### Hardware you need

- The PPP Cartesian robot (see [`docs/system_description.md`](system_description.md))
- ESP32 DevKit V1 (already mounted on the robot's controller board)
- USB-A to Micro-USB cable (for flashing the ESP32)
- A computer with a serial port monitor (Arduino IDE's Serial Monitor works)

---

## Step 1 — Understand the System

Read [`docs/system_description.md`](system_description.md) to understand:
- How the robot is wired
- What the motion parameters are
- What workspace configurations were tested

Read [`docs/shaper_theory.md`](shaper_theory.md) to understand:
- Why the belt causes vibrations
- How ZVD input shaping cancels them
- What the shaper coefficients mean

---

## Step 2 — Collect Ring-Down Data

This step characterizes the robot's vibration behavior at your configurations of interest.

### 2a. Configure the firmware

Open `firmware/1_data_collection/data_collection.ino` and update:

```cpp
// Set the test configuration
const float TEST_START_X_CM  = 22.2f;  // Start of the X move
const float TEST_START_Y_CM  = 4.5f;   // Y row to characterize
const float TEST_TARGET_X_CM = 17.2f;  // End of the X move (5 cm move)

// Set the shaper you want to test (or use a baseline with no shaping)
const float SHAPER_FN   = 11.1f;
const float SHAPER_ZETA = 0.035f;
// ...and update A1, A2, A3, T1, T2, T3 accordingly
```

### 2b. Flash to ESP32

In Arduino IDE:
1. Select board: **Tools → Board → ESP32 Dev Module**
2. Select port: **Tools → Port → (your COM port)**
3. Click **Upload**

### 2c. Collect the data

1. Open **Serial Monitor** (Tools → Serial Monitor, baud rate 115200)
2. Put the robot at position (0, 0) and press the **reset button** on the ESP32
3. The robot will run 3 trials automatically and print CSV data to Serial
4. Copy everything between `TRIAL 1` and `=== FIN TOTAL DEL ENSAYO ===` into a text file
5. Save it as a `.csv` file in `data/raw_csv/`, e.g., `x_variable_y_4_5_z_min_shaped.csv`

> **Tip:** To collect the **unshaped (baseline)** data, comment out the ZVD convolution inside `moveXMeasuredShapedZVD()` and call `baseDistanceCm()` directly instead of `shapedDistanceZVD()`. Save this as `..._unshaped.csv`.

Repeat for each Y/Z configuration you want to characterize.

---

## Step 3 — Identify fₙ and ζ

1. Open MATLAB and navigate to the `matlab/` folder
2. Update the `FILE_LIST` and `LABELS` in `identify_fn_zeta.m` to match your CSV filenames
3. Run the script:

```matlab
run('identify_fn_zeta.m')
```

4. The script will:
   - Show ring-down plots with identified peaks for each trial
   - Print a summary table with mean fₙ and ζ per configuration
   - Print the computed ZVD coefficients to paste directly into the firmware

**Example output:**
```
--- Processing: z_min, y=4.5 ---
  Trial 1: fn = 15.21 Hz,  zeta = 0.0314
  Trial 2: fn = 15.45 Hz,  zeta = 0.0329
  Trial 3: fn = 15.36 Hz,  zeta = 0.0318
  SUMMARY: fn = 15.34 ± 0.12 Hz,  zeta = 0.0320 ± 0.0008

ZVD SHAPER COEFFICIENTS:
  z_min, y=4.5   fn=15.34  zeta=0.0320  A1=0.271  A2=0.499  A3=0.230  T2=0.065s  T3=0.130s
```

---

## Step 4 — Update Shaper Coefficients

Copy the coefficients from Step 3 into the firmware lookup table in `firmware/2_scan_comparison/scan_comparison.ino`:

```cpp
const Shaper3 SHAPER_X_4_5 = {
  "(x full 4.5)",
  15.34f,         // ← updated fn
  0.032f,         // ← updated zeta
  {0.271f, 0.499f, 0.230f},  // ← A1, A2, A3
  {0.000f, 0.065f, 0.130f}   // ← T1, T2, T3
};
```

---

## Step 5 — (Optional) Verify with Simulation

Before reflashing, sanity-check the shaper with MATLAB:

1. Open `matlab/zvd_simulation.m`
2. Update `fn` and `zeta` at the top to your identified values
3. Run the script — it will show:
   - Simulated ring-down: shaped vs. unshaped
   - Predicted RMS and peak reduction
   - Robustness sweep (what happens if fₙ shifts)

---

## Step 6 — Run the Full AOI Scan

1. Flash `firmware/2_scan_comparison/scan_comparison.ino` to the ESP32
2. Put the robot at (0, 0) and press reset
3. The robot will run **two full scans**:
   - Run 1: all moves without shaping
   - Run 2: all moves with ZVD shaping (correct shaper selected per row automatically)
4. Watch the Serial Monitor to confirm the correct shaper is being selected for each move

---

## Step 7 — Process Your Results

1. Update the `CONFIG_MAP` dictionary in `python/process_results.py` with your CSV filenames
2. Update `FN_DEPLOYED` and `FN_IDENTIFIED` dictionaries with your measured values
3. Run:

```bash
cd python
python process_results.py
```

4. Figures are saved to `results/figures/`
5. A summary CSV `results_summary.csv` is also generated with all metrics

---

## Tips for Future Research

- **Try different Z positions:** The natural frequency shifts significantly with Z extension. Characterizing at more Z positions and building a finer lookup table should improve performance.
- **Try the Y axis:** This project focused on X-axis shaping. The Y axis also has a belt drive and should benefit from the same approach.
- **Closed-loop feedback:** The current system is entirely open-loop (no encoders). Adding encoder feedback would allow adaptive shaping that updates fₙ in real time.
- **Finer step resolution:** Increasing the microstepping from 1/8 to 1/16 or 1/32 would allow smoother motion profiles, at the cost of higher step frequency requirements.

---

## Common Issues

| Problem | Likely cause | Fix |
|---------|-------------|-----|
| Robot doesn't move | Wrong COM port selected, or driver not installed | Reinstall CH340 driver; check Device Manager |
| MPU6050 init fails | Bad I2C connection | Check SDA/SCL wiring; confirm AD0 is tied to GND |
| All CSV data shows az ≈ constant | Phase 2 not being captured | Increase `AFTER_MS` in firmware; check `phase == 2` filter in MATLAB |
| MATLAB says "file not found" | Working directory mismatch | Run `cd 'path/to/matlab/'` in MATLAB console before running the script |
| Very low vibration reduction | Wrong shaper frequency | Re-run identification; check if Z position matches the deployed shaper |
