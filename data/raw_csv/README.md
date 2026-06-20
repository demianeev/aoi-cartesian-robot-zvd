# Raw CSV Data

This folder contains the accelerometer ring-down data used in the paper.

## Structure

```
raw_csv/
├── unshaped/    ← baseline runs (no input shaping)
├── shaped/      ← ZVD-shaped runs (paper dataset)
└── extra/       ← additional runs not used in paper analysis
```

## Paper dataset

### Unshaped (8 configurations)
| File | Y (cm) | Z position |
|------|--------|-----------|
| x_variable_y_4_5_z_min.csv  | 4.5  | z_min |
| x_variable_y_9_5_z_min.csv  | 9.5  | z_min |
| x_variable_y_14_5_z_min.csv | 14.5 | z_min |
| x_variable_y_4_5_z_mid.csv  | 4.5  | z_mid |
| x_variable_y_9_5_z_mid.csv  | 9.5  | z_mid |
| x_variable_y_4_5_z_max.csv  | 4.5  | z_max |
| x_variable_y_9_5_z_max.csv  | 9.5  | z_max |
| x_variable_y_14_5_z_max.csv | 14.5 | z_max |

### Shaped (9 configurations)
| File | Y (cm) | Z position |
|------|--------|-----------|
| x_variable_y_4_5_z_min_shaped.csv  | 4.5  | z_min |
| x_variable_y_9_5_z_min_shaped.csv  | 9.5  | z_min |
| x_variable_y_14_5_z_min_shaped.csv | 14.5 | z_min |
| x_variable_y_4_5_z_mid_shaped.csv  | 4.5  | z_mid |
| x_variable_y_9_5_z_mid_shaped.csv  | 9.5  | z_mid |
| x_variable_y_14_5_z_mid_shaped.csv | 14.5 | z_mid |
| x_variable_y_4_5_z_max_shaped.csv  | 4.5  | z_max |
| x_variable_y_9_5_z_max_shaped.csv  | 9.5  | z_max |
| x_variable_y_14_5_z_max_shaped.csv | 14.5 | z_max |

## Known data issues

- **z_max, y=4.5**: The unshaped and shaped files are identical (same data collected twice by mistake). This configuration is excluded from the quantitative analysis but both files are kept for completeness.
- **z_min, y=9.5**: The shaped run shows slightly higher vibration than the unshaped run (RMS reduction = −8.6%, peak reduction = −57.5%). This is likely due to the shaper frequency being further from the actual resonance at this configuration. It is included in the paper as-is.
- **z_mid, y=14.5**: No unshaped baseline available for this configuration.

## CSV format

```
trial, t_us, phase, x_cm, y_cm, ax, ay, az
```

| Column | Description |
|--------|-------------|
| `trial` | Trial number (1–3 for most files; 1–6 for z_max files) |
| `t_us` | Microseconds since capture start |
| `phase` | 0 = before move, 1 = during move, **2 = post-stop ring-down** |
| `x_cm`, `y_cm` | Position command (cm) |
| `ax`, `ay`, `az` | Raw MPU6050 counts (±2g, divide by 16384 for g, ×9.81 for m/s²) |

**Primary analysis axis: `az`** (vertical — captures X-axis belt vibration)

Some older files (z_max, y=14.5) are missing the `trial`, `x_cm`, `y_cm` columns — the processing scripts handle both formats automatically.
