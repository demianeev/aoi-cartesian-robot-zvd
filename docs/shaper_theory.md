# ZVD Input Shaper — Theory Explained

This document explains the math behind the ZVD input shaper from scratch, so a future student can understand *why* it works and *how* to re-tune it.

---

## The Problem

When a belt-driven robot makes a fast move, the belt acts like a spring between the motor and the cart. After the move ends, the cart and the camera head keep oscillating at a natural frequency fₙ. The camera has to wait until this vibration damps out before an image can be taken.

This is called **residual vibration**, and it looks like this:

```
Acceleration
  ^
  |  /\    /\
  | /  \  /  \
  |/    \/    \__________  ← settles eventually
  ──────────────────────► time after stop
```

The question is: can we reshape the motion command *before* the move so that the residual vibration is already canceled by the time the robot stops?

---

## The Idea: Impulse Cancellation

Input shaping works by splitting the original command into multiple delayed, scaled copies. If you fire two impulses at the right times and amplitudes, the vibrations they cause can cancel each other out.

For a second-order system with natural frequency ωₙ and damping ratio ζ, the residual vibration from a single impulse of amplitude A fired at time tᵢ is:

```
y(t) = (A·ωₙ / √(1-ζ²)) · e^(-ζωₙ(t-tᵢ)) · sin(ωd(t-tᵢ))
```

where ωd = ωₙ√(1-ζ²) is the damped natural frequency.

If you fire N impulses, the total residual vibration is the sum of N such terms. The trick is to choose Aᵢ and tᵢ such that this sum equals zero.

---

## The ZVD Shaper (3 Impulses)

The **Zero Vibration and Derivative (ZVD)** shaper uses **3 impulses** and satisfies two constraints:
1. Residual vibration at fₙ = 0
2. *Derivative* of residual vibration with respect to frequency at fₙ = 0

Constraint 2 is what makes ZVD **robust** — it means that if your actual fₙ is a bit different from the design fₙ, the shaper still works well.

### Coefficients

Given the identified natural frequency fₙ (Hz) and damping ratio ζ:

```
ωₙ = 2π · fₙ
ωd = ωₙ · √(1 - ζ²)
K  = exp(−ζπ / √(1 − ζ²))

A₁ = 1 / (1 + 2K + K²)
A₂ = 2K / (1 + 2K + K²)
A₃ = K² / (1 + 2K + K²)

t₁ = 0
t₂ = π / ωd
t₃ = 2π / ωd
```

Note that A₁ + A₂ + A₃ = 1 (amplitudes sum to 1, so the final position is preserved).

### Shaped command

The shaped command is:

```
uₛ(t) = A₁ · u(t) + A₂ · u(t − t₂) + A₃ · u(t − t₃)
```

This adds a total delay of t₃ = 2π/ωd to the move duration. For fₙ = 11.1 Hz, ζ = 0.035:

```
t₃ = 2π / (2π · 11.1 · √(1−0.035²)) ≈ 0.090 s
```

So the move takes about 90 ms longer — a small cost relative to the settling time savings.

---

## Numerical Example (This Project)

From the experimental system identification (z_min configuration, y = 14.5 cm):

```
fₙ = 11.1 Hz    (deployed design frequency)
ζ  = 0.035

K  = exp(−0.035·π / √(1−0.035²)) = 0.8963

A₁ = 1 / (1 + 2·0.8963 + 0.8963²) = 0.2782
A₂ = 2·0.8963 / (...)              = 0.4985
A₃ = 0.8963² / (...)               = 0.2233

t₁ = 0.000 s
t₂ = 0.04507 s
t₃ = 0.09015 s
```

These exact values are what's deployed in the `SHAPER_X_14_5` struct in the firmware.

---

## Why There's a Frequency Discrepancy

The ZVD shaper was designed with fₙ = 11.1 Hz (empirically found by trial and error), but the rigorous system identification (from FFT of the ring-down signal) gave fₙ = 15.34 Hz.

This seems contradictory, but the ZVD shaper is **robust to frequency uncertainty**. The ZVD robustness window is approximately ±40% around the design frequency. The ratio 15.34 / 11.1 ≈ 1.38 falls within this window, which is why the shaper still achieves significant vibration reduction (43.7% mean RMS) even when deployed at the "wrong" frequency.

The robustness sweep in `matlab/zvd_simulation.m` plots this relationship explicitly — you can see the frequency ratio range where the shaper remains effective.

---

## How to Re-Tune the Shaper

If you reconfigure the robot (change Z height, add mass, etc.):

1. **Collect ring-down data** using `firmware/1_data_collection/`
2. **Run `matlab/identify_fn_zeta.m`** — it will output the new fₙ and ζ
3. **Compute new coefficients** using the formulas above (or let the MATLAB script do it — it prints them at the end)
4. **Update the shaper structs** in `firmware/2_scan_comparison/scan_comparison.ino`
5. **Verify with simulation** using `matlab/zvd_simulation.m` before reflashing

---

## Further Reading

- Singhose, W. (2009). *Command Shaping for Flexible Systems: A Review of the First 50 Years.* International Journal of Precision Engineering and Manufacturing, 10(4), 153–168. — The definitive review paper on input shaping.
- Singer, N.C.; Seering, W.P. (1990). *Preshaping Command Inputs to Reduce System Vibration.* Journal of Dynamic Systems, Measurement, and Control, 112, 76–82. — The original ZVD paper.
