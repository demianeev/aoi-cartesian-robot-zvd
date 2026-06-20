"""
process_results.py
Processes accelerometer CSV files, computes vibration metrics,
and generates publication figures.

PROJECT: ZVD Input Shaping for AOI Cartesian Robot
NTUST — Graduate Institute of Intelligent Manufacturing Technology

USAGE:
    cd python/
    python process_results.py

Figures are saved to ../results/figures/
Summary CSV saved to ../results/figures/results_summary.csv

DEPENDENCIES:
    pip install numpy pandas scipy matplotlib
"""

import os
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# =========================================================
# CONFIGURATION
# =========================================================

BASE      = os.path.join(os.path.dirname(__file__), '..', 'data', 'raw_csv')
FIG_DIR   = os.path.join(os.path.dirname(__file__), '..', 'results', 'figures')
os.makedirs(FIG_DIR, exist_ok=True)

FS          = 400       # Sampling rate (Hz)
LPF_CUTOFF  = 35        # Low-pass Butterworth cutoff (Hz)
LPF_ORDER   = 4
ACCEL_SCALE = 16384.0   # MPU6050 counts per g (±2g range)
G_TO_MS2    = 9.81

# Paper configurations: (z_label, y_cm, unshaped_file, shaped_file)
# Note: z_max y=4.5 excluded — unshaped and shaped files are identical (data collection error)
CONFIGS = [
    ('z_min', 4.5,  'x_variable_y_4_5_z_min.csv',   'x_variable_y_4_5_z_min_shaped.csv'),
    ('z_min', 9.5,  'x_variable_y_9_5_z_min.csv',   'x_variable_y_9_5_z_min_shaped.csv'),
    ('z_min', 14.5, 'x_variable_y_14_5_z_min.csv',  'x_variable_y_14_5_z_min_shaped.csv'),
    ('z_mid', 4.5,  'x_variable_y_4_5_z_mid.csv',   'x_variable_y_4_5_z_mid_shaped.csv'),
    ('z_mid', 9.5,  'x_variable_y_9_5_z_mid.csv',   'x_variable_y_9_5_z_mid_shaped.csv'),
    ('z_max', 9.5,  'x_variable_y_9_5_z_max.csv',   'x_variable_y_9_5_z_max_shaped.csv'),
    ('z_max', 14.5, 'x_variable_y_14_5_z_max.csv',  'x_variable_y_14_5_z_max_shaped.csv'),
]

# Deployed shaper frequencies from ESP32 firmware lookup table
FN_DEPLOYED = {
    ('z_min', 14.5): 11.1, ('z_min', 9.5): 11.3, ('z_min', 4.5): 11.6,
    ('z_mid', 14.5): 11.1, ('z_mid', 9.5): 11.3, ('z_mid', 4.5): 11.6,
    ('z_max', 14.5): 11.1, ('z_max', 9.5): 11.3, ('z_max', 4.5): 11.6,
}

# Identified natural frequencies from system ID (identify_fn_zeta.m)
FN_IDENTIFIED = {
    ('z_min', 14.5): 15.34, ('z_min', 9.5): 15.34, ('z_min', 4.5): 15.34,
    ('z_mid', 14.5): 15.34, ('z_mid', 9.5): 15.34, ('z_mid', 4.5): 15.34,
    ('z_max', 14.5): 14.9,  ('z_max', 9.5): 15.6,  ('z_max', 4.5): 15.6,
}

Z_COLORS  = {'z_min': '#1f77b4', 'z_mid': '#ff7f0e', 'z_max': '#d62728'}
Z_MARKERS = {'z_min': 'o',       'z_mid': 's',        'z_max': '^'}

# =========================================================
# HELPERS
# =========================================================

def load_clean(fpath):
    """Load CSV, drop unnamed columns, normalise trial column."""
    df = pd.read_csv(fpath)
    df.columns = [c.strip().lower() for c in df.columns]
    df = df[[c for c in df.columns if not c.startswith('unnamed')]]
    if 'trial' not in df.columns:
        df['trial'] = 1
    df = df.dropna(subset=['trial'])
    df['trial'] = df['trial'].astype(int)
    return df

def process_ringdown(df):
    """Average phase-2 ring-down across trials, filter, compute metrics."""
    sub = df[df['phase'] == 2].copy()
    trials = sorted(sub['trial'].unique())
    all_az = []
    for trial in trials:
        tr = sub[sub['trial'] == trial]
        az_raw = tr['az'].values.astype(float)
        az_ms2 = (az_raw / ACCEL_SCALE) * G_TO_MS2
        dc = np.mean(az_ms2[int(0.8 * len(az_ms2)):])
        az_ms2 -= dc
        b, a = butter(LPF_ORDER, LPF_CUTOFF / (FS / 2), btype='low')
        az_filt = filtfilt(b, a, az_ms2)
        all_az.append(az_filt)
    min_len = min(len(a) for a in all_az)
    az_mean = np.mean([a[:min_len] for a in all_az], axis=0)
    t       = np.arange(min_len) / FS
    Epeak   = float(np.max(np.abs(az_mean)))
    Erms    = float(np.sqrt(np.mean(az_mean ** 2)))
    thresh  = 0.05 * Epeak
    above   = np.where(np.abs(az_mean) > thresh)[0]
    ts      = float(t[above[-1]]) if len(above) > 0 else 0.0
    return t, az_mean, Epeak, Erms, ts, thresh

# =========================================================
# MAIN LOOP
# =========================================================

results  = []
all_data = {}

for z, y, fu, fs in CONFIGS:
    pu = os.path.join(BASE, 'unshaped', fu)
    ps = os.path.join(BASE, 'shaped',   fs)
    if not os.path.isfile(pu) or not os.path.isfile(ps):
        print(f"  SKIP {z}, y={y}: file not found")
        continue
    df_u = load_clean(pu)
    df_s = load_clean(ps)
    t_u, az_u, Ep_u, Er_u, ts_u, thresh = process_ringdown(df_u)
    t_s, az_s, Ep_s, Er_s, ts_s, _      = process_ringdown(df_s)

    peak_red   = 100 * (1 - Ep_s / Ep_u)
    rms_red    = 100 * (1 - Er_s / Er_u)
    fn_dep     = FN_DEPLOYED.get((z, y))
    fn_id      = FN_IDENTIFIED.get((z, y))
    freq_ratio = (fn_id / fn_dep) if fn_dep else None

    results.append({'z': z, 'y_cm': y,
                    'Epeak_unshaped': Ep_u, 'Epeak_shaped': Ep_s, 'peak_red_%': peak_red,
                    'Erms_unshaped':  Er_u, 'Erms_shaped':  Er_s, 'rms_red_%':  rms_red,
                    'settling_unshaped_s': ts_u, 'settling_shaped_s': ts_s,
                    'fn_deployed': fn_dep, 'fn_identified': fn_id, 'freq_ratio': freq_ratio})
    all_data[(z, y)] = {'t_u': t_u, 'az_u': az_u,
                        't_s': t_s, 'az_s': az_s, 'thresh': thresh}
    print(f"  {z}, y={y:<5}  RMS red={rms_red:+.1f}%  Peak red={peak_red:+.1f}%")

# =========================================================
# FIGURE 1 — 3-panel representative ring-downs
# =========================================================

rep_configs = [('z_min', 14.5), ('z_mid', 4.5), ('z_max', 14.5)]
fig, axes = plt.subplots(1, 3, figsize=(14, 4))
for ax, (z, y) in zip(axes, rep_configs):
    if (z, y) not in all_data:
        continue
    d = all_data[(z, y)]
    r = next(r for r in results if r['z'] == z and r['y_cm'] == y)
    ax.plot(d['t_u'], d['az_u'], 'b--', lw=1.3, label='Unshaped')
    ax.plot(d['t_s'], d['az_s'], 'g-',  lw=1.3, label='ZVD Shaped')
    ax.axhline( d['thresh'], color='k', ls=':', lw=0.8)
    ax.axhline(-d['thresh'], color='k', ls=':', lw=0.8)
    ax.set_title(f'{z}, y={y} cm\nRMS red. {r["rms_red_%"]:.1f}%', fontsize=10)
    ax.set_xlabel('Time after stop (s)')
    ax.set_ylabel('Residual accel. (m/s²)')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.35); ax.set_xlim([0, 1.5])
fig.suptitle('Post-Stop Residual Vibration — Representative Cases per Z Position', fontsize=12)
fig.tight_layout()
fig.savefig(os.path.join(FIG_DIR, 'fig_ringdown_3panel.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Saved fig_ringdown_3panel.png")

# =========================================================
# FIGURE 2 — Individual ring-down per configuration
# =========================================================

for z, y, *_ in CONFIGS:
    if (z, y) not in all_data:
        continue
    d = all_data[(z, y)]
    r = next(r for r in results if r['z'] == z and r['y_cm'] == y)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(d['t_u'], d['az_u'], 'b--', lw=1.5, label='Unshaped')
    ax.plot(d['t_s'], d['az_s'], 'g-',  lw=1.5, label='ZVD Shaped')
    ax.axhline( d['thresh'], color='k', ls=':', lw=0.9,
                label=f'±{d["thresh"]:.3f} m/s² (5% threshold)')
    ax.axhline(-d['thresh'], color='k', ls=':', lw=0.9)
    note = f'RMS red: {r["rms_red_%"]:.1f}%  |  Peak red: {r["peak_red_%"]:.1f}%'
    ax.set_title(f'Post-Stop Residual Vibration — Unshaped vs ZVD Shaped\n'
                 f'(X-axis, y = {y} cm, {z})  —  {note}')
    ax.set_xlabel('Time after stop (s)')
    ax.set_ylabel('Residual acceleration (m/s²)')
    ax.legend(); ax.grid(True, alpha=0.35); ax.set_xlim([0, 1.5])
    fname = f'ringdown_{z}_y{str(y).replace(".", "p")}.png'
    fig.savefig(os.path.join(FIG_DIR, fname), dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved {fname}")

# =========================================================
# FIGURE 3 — Robustness scatter
# =========================================================

fig, ax = plt.subplots(figsize=(9, 5))
for z_lbl in ['z_min', 'z_mid', 'z_max']:
    grp = [r for r in results if r['z'] == z_lbl and r['freq_ratio'] is not None]
    if not grp:
        continue
    xs = [r['freq_ratio']  for r in grp]
    ys = [r['rms_red_%']   for r in grp]
    ax.scatter(xs, ys, c=Z_COLORS[z_lbl], marker=Z_MARKERS[z_lbl],
               s=120, label=z_lbl, zorder=3)
    for r in grp:
        ax.annotate(f"{r['z']}\ny={r['y_cm']}",
                    (r['freq_ratio'], r['rms_red_%']),
                    textcoords='offset points', xytext=(6, 4), fontsize=8)
ax.axvline(1.0, color='k', ls='--', lw=1, label='Design frequency ratio = 1')
ax.axhline(0,   color='gray', ls='--', lw=0.8)
ax.set_xlabel('Frequency ratio  fn_identified / fn_deployed')
ax.set_ylabel('RMS residual vibration reduction (%)')
ax.set_title('ZVD Shaper Effectiveness vs Frequency Ratio\n(All configurations)')
ax.legend(); ax.grid(True, alpha=0.3)
fig.tight_layout()
fig.savefig(os.path.join(FIG_DIR, 'fig_robustness_scatter.png'), dpi=150, bbox_inches='tight')
plt.close()
print("Saved fig_robustness_scatter.png")

# =========================================================
# SUMMARY CSV
# =========================================================

df_metrics = pd.DataFrame(results)
df_metrics.to_csv(os.path.join(FIG_DIR, 'results_summary.csv'), index=False)

print("\n=== RESULTS SUMMARY ===")
print(df_metrics[['z','y_cm','rms_red_%','peak_red_%',
                   'settling_unshaped_s','settling_shaped_s']].to_string(index=False))
mean_rms  = df_metrics['rms_red_%'].mean()
max_peak  = df_metrics['peak_red_%'].max()
print(f"\nMean RMS reduction:  {mean_rms:.1f}%")
print(f"Max peak reduction:  {max_peak:.1f}%")
print("\nDone.")
