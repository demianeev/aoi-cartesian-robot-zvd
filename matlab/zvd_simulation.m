% =========================================================================
% zvd_simulation.m
% ZVD Input Shaper Simulation — Base-Excitation Second-Order ODE Model
%
% PROJECT: ZVD Input Shaping for AOI Cartesian Robot
% NTUST — Graduate Institute of Intelligent Manufacturing Technology
%
% HOW TO USE:
%   1. Set fn, zeta, and the trapezoidal move parameters below
%   2. Run to see simulated ring-down (shaped vs. unshaped) and robustness sweep
%   3. The script also prints ZVD coefficients to paste into the firmware
%
% MODEL:
%   The belt-drive flexible mode is treated as a second-order base-excitation
%   system driven by the trapezoidal cart acceleration ü(t).
%   Equation of motion (post-stop free ring-down):
%       ẍ + 2ζωₙẋ + ωₙ²x = ü(t)
%   where x is the relative tip displacement.
% =========================================================================

clear; clc; close all;

%% =========================================================
% SECTION 1: PARAMETERS — Update these after running identify_fn_zeta.m
%% =========================================================

fn   = 11.1;    % Design natural frequency (Hz) — from system ID or empirical tuning
zeta = 0.035;   % Damping ratio — from system ID

% Trapezoidal move parameters (must match firmware)
D    = 19.2e-2;  % Move distance (m) — 19.2 cm
Vmax = 0.30;     % Max velocity (m/s) — 30 cm/s
Amax = 2.20;     % Acceleration (m/s²) — 220 cm/s²

% Simulation time after stop (s)
T_POST_STOP = 1.5;

% Robustness sweep range
FREQ_RATIO_RANGE = linspace(0.5, 1.8, 100);  % fn_actual / fn_design

%% =========================================================
% SECTION 2: DERIVED PARAMETERS & ZVD COEFFICIENTS
%% =========================================================

wn = 2*pi*fn;
wd = wn * sqrt(1 - zeta^2);
K  = exp(-zeta * pi / sqrt(1 - zeta^2));

denom = 1 + 2*K + K^2;
A1 = 1/denom;
A2 = 2*K/denom;
A3 = K^2/denom;
T1 = 0;
T2 = pi/wd;
T3 = 2*pi/wd;

fprintf('=== ZVD Shaper Coefficients ===\n');
fprintf('fn = %.2f Hz,  zeta = %.4f\n', fn, zeta);
fprintf('A1=%.6f,  A2=%.6f,  A3=%.6f\n', A1, A2, A3);
fprintf('T1=%.6f,  T2=%.6f,  T3=%.6f  (seconds)\n\n', T1, T2, T3);

%% =========================================================
% SECTION 3: TRAPEZOIDAL PROFILE
%% =========================================================

tAcc = Vmax / Amax;
dAcc = 0.5 * Amax * tAcc^2;

if 2*dAcc >= D
    % Triangular profile (short move)
    tAcc  = sqrt(D / Amax);
    Vpeak = Amax * tAcc;
    tCruise = 0;
else
    Vpeak   = Vmax;
    tCruise = (D - 2*dAcc) / Vmax;
end

tTotal = 2*tAcc + tCruise;
fprintf('Trapezoidal profile: D=%.3f m, V=%.2f m/s, A=%.2f m/s², tTotal=%.3f s\n', ...
    D, Vpeak, Amax, tTotal);

% Acceleration command function
accelCmd = @(t) trapezAccel(t, tAcc, tCruise, tTotal, Amax);

% Shaped acceleration command (ZVD convolution)
shapedAccelCmd = @(t) A1*accelCmd(t-T1) + A2*accelCmd(t-T2) + A3*accelCmd(t-T3);

%% =========================================================
% SECTION 4: ODE SIMULATION (forced phase during move, then free ring-down)
%% =========================================================

dt = 1e-4;  % Integration time step (s)

% --- Unshaped ---
[x0_u, v0_u] = integrateODE(accelCmd, tTotal, wn, zeta, dt);
[t_rd, az_u]  = ringDown(x0_u, v0_u, wn, zeta, T_POST_STOP, dt);

% --- ZVD Shaped ---
tTotal_zvd   = tTotal + T3;
[x0_s, v0_s] = integrateODE(shapedAccelCmd, tTotal_zvd, wn, zeta, dt);
[~, az_s]    = ringDown(x0_s, v0_s, wn, zeta, T_POST_STOP, dt);

%% =========================================================
% SECTION 5: METRICS
%% =========================================================

Epeak_u  = max(abs(az_u));
Erms_u   = rms(az_u);
Epeak_s  = max(abs(az_s));
Erms_s   = rms(az_s);

thresh = 0.05 * Epeak_u;  % 5% settling threshold
ts_u = settlingTime(t_rd, az_u, thresh);
ts_s = settlingTime(t_rd, az_s, thresh);

fprintf('\n=== Simulation Results ===\n');
fprintf('Unshaped:  E_peak=%.4f m/s²,  E_rms=%.4f m/s²,  settling=%.3f s\n', Epeak_u, Erms_u, ts_u);
fprintf('ZVD:       E_peak=%.4f m/s²,  E_rms=%.4f m/s²,  settling=%.3f s\n', Epeak_s, Erms_s, ts_s);
fprintf('Reduction: E_peak: %.1f%%,  E_rms: %.1f%%\n', ...
    100*(1-Epeak_s/Epeak_u), 100*(1-Erms_s/Erms_u));

%% =========================================================
% SECTION 6: PLOT — Ring-down comparison
%% =========================================================

figure('Name', 'Simulated Ring-Down: Unshaped vs ZVD Shaped', 'Position', [100 100 900 400]);
plot(t_rd, az_u, 'b--', 'LineWidth', 1.5, 'DisplayName', 'Unshaped'); hold on;
plot(t_rd, az_s, 'g-',  'LineWidth', 1.5, 'DisplayName', 'ZVD Shaped');
yline(thresh, 'k:', sprintf('±%.3f m/s² (5%% threshold)', thresh), 'LabelVerticalAlignment', 'bottom');
yline(-thresh, 'k:');
xlabel('Time after stop (s)'); ylabel('Residual acceleration (m/s²)');
title(sprintf('Simulated Post-Stop Ring-Down — fn = %.1f Hz, \\zeta = %.3f', fn, zeta));
legend('Location', 'northeast'); grid on;

%% =========================================================
% SECTION 7: ROBUSTNESS SWEEP
%% =========================================================

% For each actual frequency (as ratio to design frequency),
% compute residual vibration with the ZVD shaper designed at fn
rms_unshaped = zeros(size(FREQ_RATIO_RANGE));
rms_shaped   = zeros(size(FREQ_RATIO_RANGE));

for i = 1:length(FREQ_RATIO_RANGE)
    fn_actual  = fn * FREQ_RATIO_RANGE(i);
    wn_actual  = 2*pi*fn_actual;

    [x0u, v0u] = integrateODE(accelCmd, tTotal, wn_actual, zeta, dt);
    [~, az_u_i] = ringDown(x0u, v0u, wn_actual, zeta, T_POST_STOP, dt);

    [x0s, v0s] = integrateODE(shapedAccelCmd, tTotal_zvd, wn_actual, zeta, dt);
    [~, az_s_i] = ringDown(x0s, v0s, wn_actual, zeta, T_POST_STOP, dt);

    rms_unshaped(i) = rms(az_u_i);
    rms_shaped(i)   = rms(az_s_i);
end

reduction_pct = 100 * (1 - rms_shaped ./ rms_unshaped);

figure('Name', 'ZVD Robustness Sweep', 'Position', [100 550 900 400]);
plot(FREQ_RATIO_RANGE, reduction_pct, 'b-', 'LineWidth', 2);
xline(1.0, 'k--', 'Design point', 'LabelVerticalAlignment', 'top');
yline(0, 'r:', 'No improvement');
xlabel('Frequency ratio  f_n actual / f_n design');
ylabel('RMS residual vibration reduction (%)');
title('ZVD Shaper Robustness to Natural Frequency Uncertainty');
grid on; ylim([-20, 100]);

%% =========================================================
% LOCAL FUNCTIONS
%% =========================================================

function a = trapezAccel(t, tAcc, tCruise, tTotal, Amax)
    % Returns trapezoidal acceleration profile value at time t
    a = zeros(size(t));
    for i = 1:numel(t)
        ti = t(i);
        if ti < 0
            a(i) = 0;
        elseif ti < tAcc
            a(i) = Amax;
        elseif ti < (tAcc + tCruise)
            a(i) = 0;
        elseif ti < tTotal
            a(i) = -Amax;
        else
            a(i) = 0;
        end
    end
end

function [x_stop, v_stop] = integrateODE(forceFcn, tEnd, wn, zeta, dt)
    % Integrate the forced second-order ODE over [0, tEnd]
    % Returns displacement and velocity at tEnd (initial conditions for ring-down)
    t  = 0:dt:tEnd;
    x  = 0; v = 0;
    for i = 1:length(t)-1
        ti = t(i);
        f  = forceFcn(ti);
        a  = f - 2*zeta*wn*v - wn^2*x;
        v  = v + a*dt;
        x  = x + v*dt;
    end
    x_stop = x;
    v_stop = v;
end

function [t_rd, az_rd] = ringDown(x0, v0, wn, zeta, T_post, dt)
    % Simulate free ring-down from initial conditions x0, v0
    t_rd = 0:dt:T_post;
    x = x0; v = v0;
    az_rd = zeros(size(t_rd));
    for i = 1:length(t_rd)
        az_rd(i) = -(wn^2 * x + 2*zeta*wn*v);
        a = -2*zeta*wn*v - wn^2*x;
        v = v + a*dt;
        x = x + v*dt;
    end
end

function ts = settlingTime(t, y, thresh)
    % Returns the last time the signal exceeds thresh
    idx = find(abs(y) > thresh, 1, 'last');
    if isempty(idx)
        ts = 0;
    else
        ts = t(idx);
    end
end
