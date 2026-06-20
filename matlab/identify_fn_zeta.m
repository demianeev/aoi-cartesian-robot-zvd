% =========================================================================
% identify_fn_zeta.m
% Natural Frequency and Damping Ratio Identification
% from Post-Stop Ring-Down Accelerometer Data
%
% PROJECT: ZVD Input Shaping for AOI Cartesian Robot
% NTUST — Graduate Institute of Intelligent Manufacturing Technology
%
% HOW TO USE:
%   1. Place your CSV files from the data collection firmware in data/raw_csv/
%   2. Update the FILE_LIST and LABELS below to match your files
%   3. Run this script — it will print identified fn and zeta for each config
%   4. Use those values to update the ZVD coefficients in the firmware
%
% CSV FORMAT EXPECTED:
%   trial, t_us, phase, x_cm, y_cm, ax, ay, az
%   phase: 0 = before, 1 = during, 2 = after (ring-down)
%   az units: raw ADC counts (±2g, divide by 16384 for g)
% =========================================================================

clear; clc; close all;

%% =========================================================
% SECTION 1: CONFIGURATION — Update these to match your files
%% =========================================================

DATA_DIR = '../data/raw_csv/';  % Path to CSV folder (relative to this script)

% List your CSV files and give each a label
FILE_LIST = {
    'x_variable_y_4_5_z_min.csv',
    'x_variable_y_9_5_z_min.csv',
    'x_variable_y_14_5_z_min.csv',
    'x_variable_y_4_5_z_mid.csv',
    'x_variable_y_9_5_z_mid.csv',
    'x_variable_y_4_5_z_max.csv',
    'x_variable_y_9_5_z_max.csv',
    'x_variable_y_14_5_z_max.csv',
};

LABELS = {
    'z_min, y=4.5',
    'z_min, y=9.5',
    'z_min, y=14.5',
    'z_mid, y=4.5',
    'z_mid, y=9.5',
    'z_max, y=4.5',
    'z_max, y=9.5',
    'z_max, y=14.5',
};

% Signal processing parameters
FS          = 400;       % Sampling frequency (Hz) — matches firmware SAMPLE_PERIOD_US = 2500
LPF_CUTOFF  = 35;        % Low-pass filter cutoff (Hz) — removes high-freq noise
LPF_ORDER   = 4;         % Butterworth filter order
ACCEL_SCALE = 16384.0;   % MPU6050 raw counts per g (±2g range)
G_TO_MS2    = 9.81;      % Convert g to m/s²

% Logarithmic decrement: use this many peaks
N_PEAKS_FOR_ZETA = 5;

%% =========================================================
% SECTION 2: PROCESS EACH FILE
%% =========================================================

results = struct('label', {}, 'fn_mean', {}, 'fn_std', {}, ...
                 'zeta_mean', {}, 'zeta_std', {}, 'fn_all', {}, 'zeta_all', {});

for fileIdx = 1:length(FILE_LIST)
    fname = FILE_LIST{fileIdx};
    fpath = fullfile(DATA_DIR, fname);
    label = LABELS{fileIdx};

    if ~isfile(fpath)
        fprintf('WARNING: File not found — %s. Skipping.\n', fname);
        continue;
    end

    fprintf('\n--- Processing: %s ---\n', label);

    % Read CSV
    try
        T = readtable(fpath, 'VariableNamingRule', 'preserve');
    catch
        fprintf('ERROR reading %s. Skipping.\n', fname);
        continue;
    end

    % Normalize column names to lowercase
    T.Properties.VariableNames = lower(T.Properties.VariableNames);

    % Get unique trials
    if ismember('trial', T.Properties.VariableNames)
        trials = unique(T.trial);
    else
        trials = 1;
        T.trial = ones(height(T), 1);
    end

    fn_cells   = {};
    zeta_cells = {};

    for trialIdx = 1:length(trials)
        trialNum = trials(trialIdx);
        Tsub = T(T.trial == trialNum & T.phase == 2, :);  % phase 2 = post-stop ring-down

        if height(Tsub) < 20
            fprintf('  Trial %d: too few post-stop samples, skipping.\n', trialNum);
            continue;
        end

        % Convert time to seconds (t_us is in microseconds since capture start)
        t_raw = double(Tsub.t_us) * 1e-6;
        t     = t_raw - t_raw(1);  % Start from zero

        % Convert az from raw counts to m/s²
        az_raw = double(Tsub.az);
        az_ms2 = (az_raw / ACCEL_SCALE) * G_TO_MS2;

        % Remove DC offset (steady-state mean of last 20% of signal)
        tail_start = round(0.8 * length(az_ms2));
        az_offset  = mean(az_ms2(tail_start:end));
        az         = az_ms2 - az_offset;

        % Low-pass Butterworth filter
        [b, a] = butter(LPF_ORDER, LPF_CUTOFF / (FS/2), 'low');
        az_filt = filtfilt(b, a, az);

        %% --- Natural Frequency: FFT peak ---
        N   = length(az_filt);
        f   = (0:N-1) * (FS / N);
        Az  = abs(fft(az_filt));
        Az  = Az(1:floor(N/2));
        f   = f(1:floor(N/2));

        % Only look between 5 and 30 Hz
        mask = (f >= 5) & (f <= 30);
        [~, imax] = max(Az(mask));
        f_subset  = f(mask);
        fn_trial  = f_subset(imax);

        %% --- Damping Ratio: Logarithmic Decrement ---
        % Find positive peaks of the filtered signal
        [pks, locs] = findpeaks(az_filt, 'MinPeakProminence', 0.01);

        if length(pks) >= N_PEAKS_FOR_ZETA + 1
            pk1  = pks(1);
            pkN1 = pks(N_PEAKS_FOR_ZETA + 1);
            delta = (1 / N_PEAKS_FOR_ZETA) * log(pk1 / pkN1);
            zeta_trial = delta / sqrt(4*pi^2 + delta^2);
        else
            fprintf('  Trial %d: not enough peaks for zeta estimation.\n', trialNum);
            zeta_trial = NaN;
        end

        fprintf('  Trial %d: fn = %.2f Hz,  zeta = %.4f\n', trialNum, fn_trial, zeta_trial);

        fn_cells{end+1}   = fn_trial;
        zeta_cells{end+1} = zeta_trial;

        %% --- Optional plot: ring-down for this trial ---
        figure('Name', sprintf('%s — Trial %d', label, trialNum), 'Visible', 'on');
        plot(t, az_filt, 'b-', 'LineWidth', 1.2); hold on;
        if ~isempty(locs)
            plot(t(locs), az_filt(locs), 'rv', 'MarkerSize', 6, 'DisplayName', 'Peaks');
        end
        xlabel('Time (s)'); ylabel('Filtered az (m/s²)');
        title(sprintf('%s | Trial %d | fn=%.1f Hz, \\zeta=%.4f', label, trialNum, fn_trial, zeta_trial));
        grid on; legend({'Filtered signal', 'Peaks'});
    end

    % Aggregate over trials
    fn_all   = cell2mat(fn_cells);
    zeta_all = cell2mat(zeta_cells);
    zeta_all = zeta_all(~isnan(zeta_all));

    if isempty(fn_all)
        fprintf('  No valid trials found for %s\n', label);
        continue;
    end

    r.label     = label;
    r.fn_mean   = mean(fn_all);
    r.fn_std    = std(fn_all);
    r.zeta_mean = mean(zeta_all);
    r.zeta_std  = std(zeta_all);
    r.fn_all    = fn_all;
    r.zeta_all  = zeta_all;
    results(end+1) = r;

    fprintf('  SUMMARY: fn = %.2f ± %.2f Hz,  zeta = %.4f ± %.4f\n', ...
        r.fn_mean, r.fn_std, r.zeta_mean, r.zeta_std);
end

%% =========================================================
% SECTION 3: FINAL SUMMARY TABLE
%% =========================================================
fprintf('\n\n========================================\n');
fprintf('FINAL SUMMARY — All Configurations\n');
fprintf('========================================\n');
fprintf('%-25s  %8s  %8s  %8s  %8s\n', 'Label', 'fn_mean', 'fn_std', 'zeta_mean', 'zeta_std');
fprintf('%s\n', repmat('-', 1, 70));

for i = 1:length(results)
    r = results(i);
    fprintf('%-25s  %8.2f  %8.2f  %8.4f  %8.4f\n', ...
        r.label, r.fn_mean, r.fn_std, r.zeta_mean, r.zeta_std);
end

%% =========================================================
% SECTION 4: COMPUTE ZVD COEFFICIENTS FOR THE IDENTIFIED PARAMETERS
%% =========================================================
if ~isempty(results)
    fprintf('\n\n========================================\n');
    fprintf('ZVD SHAPER COEFFICIENTS\n');
    fprintf('(use these values in the ESP32 firmware)\n');
    fprintf('========================================\n');
    fprintf('%-25s  %6s  %6s  %6s  %6s  %6s  %8s  %8s\n', ...
        'Label', 'fn', 'zeta', 'A1', 'A2', 'A3', 'T2(s)', 'T3(s)');
    fprintf('%s\n', repmat('-', 1, 90));

    for i = 1:length(results)
        r   = results(i);
        fn  = r.fn_mean;
        z   = r.zeta_mean;
        wn  = 2*pi*fn;
        wd  = wn * sqrt(1 - z^2);
        K   = exp(-z*pi / sqrt(1 - z^2));
        den = 1 + 2*K + K^2;
        A1  = 1/den;
        A2  = 2*K/den;
        A3  = K^2/den;
        T2  = pi/wd;
        T3  = 2*pi/wd;
        fprintf('%-25s  %6.2f  %6.4f  %6.4f  %6.4f  %6.4f  %8.5f  %8.5f\n', ...
            r.label, fn, z, A1, A2, A3, T2, T3);
    end
end
