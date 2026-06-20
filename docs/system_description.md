# System Description

## Overview

This is a **PPP (Prismatic-Prismatic-Prismatic) Cartesian robot** built for Automated Optical Inspection (AOI) of printed circuit boards (PCBs). All three axes move linearly and are orthogonal to each other.

The robot was built and programmed as a capstone project at NTUST. The core research question was: *can we reduce the camera head's settling time after each move — without changing the hardware — by shaping the motion command before sending it to the motors?*

The answer is yes, using **ZVD input shaping**.

---

## Hardware Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ESP32 (DevKit V1)                        │
│                                                                 │
│  GPIO 26 ──STEP──► A4988 ──► NEMA 17 ──► Belt ──► X Cart      │
│  GPIO 27 ──DIR───►                                              │
│                                                                 │
│  GPIO 25 ──STEP──► A4988 ──► NEMA 17 ──► Belt ──► Y Cart      │
│  GPIO 33 ──DIR───►                                              │
│                                                                 │
│  GPIO 21 (SDA) ─┐                                              │
│  GPIO 22 (SCL) ─┴──► MPU6050 ──► (mounted on camera head)     │
└─────────────────────────────────────────────────────────────────┘
```

### Axes

| Axis | Drive Mechanism | Motor | Purpose |
|------|----------------|-------|---------|
| X | GT2 belt + 20-tooth pulley | NEMA 17 (17HS8401) | Horizontal scan (fast axis) |
| Y | GT2 belt + 20-tooth pulley | NEMA 17 (17HS8401) | Row-to-row stepping |
| Z | Ball screw | NEMA 17 (17HS8401) | Camera height adjustment |

### Mechanical Parameters

| Parameter | Value |
|-----------|-------|
| Belt pitch | 2 mm (GT2) |
| Pulley teeth | 20 |
| Motor steps/rev | 200 (full step) |
| Microstepping | 1/8 (A4988 default) |
| **Steps per cm** | **400 steps/cm** |
| Workspace (X) | 0 – 25 cm |
| Workspace (Y) | 0 – 19.2 cm |

> **Steps per cm calculation:** `(200 steps/rev × 8 microsteps) / (20 teeth × 0.2 cm/tooth) = 400 steps/cm`

### Accelerometer

The **MPU6050** is mounted directly on the camera head assembly on the Z-axis arm. It measures the residual vibration of the camera after each X-axis move.

| Parameter | Value |
|-----------|-------|
| Interface | I2C (address 0x68) |
| Range | ±2g |
| Scale factor | 16384 counts/g |
| Sampling rate | 400 Hz (2500 µs period) |
| Primary axis used | az (vertical, captures X-axis belt vibration) |

---

## Motion Profiles

All X and Y scan moves use a **trapezoidal velocity profile**:

```
Velocity
  Vmax ─────────────────────
       /                    \
      /                      \
─────/                        \─────
   tAcc    tCruise    tAcc
```

| Profile | Velocity (cm/s) | Acceleration (cm/s²) | Used for |
|---------|----------------|---------------------|---------|
| Fast    | 30.0           | 220.0               | Scan moves (shaped/unshaped) |
| Slow    | 14.0           | 90.0                | Entry/exit, repositioning     |

---

## Signal Chain for ZVD Input Shaping

```
Reference command u(t)
        │
        ▼
  ┌─────────────┐        Convolution with 3-impulse ZVD sequence:
  │ ZVD Shaper  │   uₛ(t) = A₁·u(t) + A₂·u(t-T₂) + A₃·u(t-T₃)
  └─────────────┘
        │
        ▼
  Shaped command uₛ(t)   ← sent to stepper driver as step pulses
        │
        ▼
  NEMA 17 stepper ──► Belt ──► X Cart ──► (camera head)
                                               │
                                           MPU6050
                                        (measures az)
```

The shaping is done **in software on the ESP32** — the `shapedPosCm()` function evaluates the shaped position command in real-time on every loop iteration and emits step pulses accordingly. No additional hardware is required.

---

## Workspace Configurations Tested

The robot was characterized across a **3×3 grid** of Y × Z configurations:

| Z position | Description | Physical meaning |
|------------|-------------|-----------------|
| z_min | Minimum height | Maximum Z-arm extension → lowest structural stiffness |
| z_mid | Middle height  | Intermediate extension |
| z_max | Maximum height | Minimum extension → highest structural stiffness |

| Y position | Description |
|------------|-------------|
| y = 4.5 cm | Y rail closest to motor |
| y = 9.5 cm | Middle of Y range |
| y = 14.5 cm | Y rail farthest from motor |

Not all 9 combinations were characterized — see the paper for the exact tested configurations.
