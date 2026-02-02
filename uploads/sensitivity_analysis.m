%% MCM 2026 Problem A - Sensitivity & Robustness Analysis
% =========================================================================
% This module performs comprehensive sensitivity analysis on the smartphone
% power simulation model to evaluate:
%   1. Parameter sensitivity (local & global)
%   2. Model robustness under parameter uncertainty
%   3. Usage pattern variation effects
%   4. Monte Carlo uncertainty quantification
% =========================================================================

clear; clc; close all;

fprintf('=========================================================\n');
fprintf('    Smartphone Power Model - Sensitivity Analysis\n');
fprintf('=========================================================\n\n');

%% ========================================================================
%  SECTION 1: BASELINE MODEL CONFIGURATION
% ========================================================================

% --- 1.1 Base Parameters (Reference Values) ---
BaseParams.Batt_Q_design = 5000;          % mAh
BaseParams.Batt_V_nom = 3.85;             % V
BaseParams.Batt_R_internal = 0.05;        % Ohm
BaseParams.PMIC_eff = 0.92;               % Efficiency

BaseParams.SoC_C_eff = 1.2e-9;            % Effective capacitance
BaseParams.SoC_V_min = 0.65;              % V
BaseParams.SoC_V_max = 1.05;              % V
BaseParams.SoC_I_leak = 0.005;            % A

BaseParams.Disp_P_static = 0.050;         % W
BaseParams.Disp_P_dyn_slope = 0.0005;     % W/Hz
BaseParams.Disp_Beta_panel = 2.5e-3;      % W/nit
BaseParams.Disp_L_max = 1200;             % nits

BaseParams.Conn_5G_idle = 0.080;          % W
BaseParams.Conn_5G_active_high = 1.200;   % W
BaseParams.Conn_WiFi_active = 0.400;      % W
BaseParams.Conn_BT_active = 0.040;        % W
BaseParams.Conn_GPS = 0.150;              % W

BaseParams.Therm_C_th = 850;              % J/K
BaseParams.Therm_R_th = 18;               % K/W
BaseParams.Therm_T_throttle = 316.15;     % K

% Parameter names for display
ParamNames = fieldnames(BaseParams);
NumParams = length(ParamNames);

fprintf('Base Model Parameters: %d parameters defined\n\n', NumParams);

%% ========================================================================
%  SECTION 2: LOCAL SENSITIVITY ANALYSIS (ONE-AT-A-TIME)
% ========================================================================

fprintf('----------------------------------------------------------\n');
fprintf('SECTION 2: Local Sensitivity Analysis (OAT Method)\n');
fprintf('----------------------------------------------------------\n\n');

% Perturbation range: +/- 20%
perturbation = 0.20;
N_trials = 5;  % Repeat for statistical stability

% Baseline run
rng(2026);
[E_base, P_avg_base, P_peak_base, Life_base, T_max_base] = run_simulation(BaseParams);

fprintf('Baseline Results:\n');
fprintf('  Total Energy: %.2f Wh\n', E_base);
fprintf('  Avg Power: %.1f mW\n', P_avg_base);
fprintf('  Peak Power: %.1f mW\n', P_peak_base);
fprintf('  Battery Life: %.1f hours\n', Life_base);
fprintf('  Max Temp: %.1f C\n\n', T_max_base);

% Sensitivity indices storage
Sensitivity.Energy = zeros(NumParams, 1);
Sensitivity.AvgPower = zeros(NumParams, 1);
Sensitivity.PeakPower = zeros(NumParams, 1);
Sensitivity.BatteryLife = zeros(NumParams, 1);
Sensitivity.MaxTemp = zeros(NumParams, 1);

fprintf('Computing local sensitivities (OAT)...\n');

for i = 1:NumParams
    param_name = ParamNames{i};
    base_val = BaseParams.(param_name);
    
    E_high_arr = zeros(1, N_trials);
    E_low_arr = zeros(1, N_trials);
    
    for trial = 1:N_trials
        rng(2026 + trial);
        
        % High perturbation
        params_high = BaseParams;
        params_high.(param_name) = base_val * (1 + perturbation);
        [E_high, ~, ~, ~, ~] = run_simulation(params_high);
        E_high_arr(trial) = E_high;
        
        % Low perturbation
        params_low = BaseParams;
        params_low.(param_name) = base_val * (1 - perturbation);
        [E_low, ~, ~, ~, ~] = run_simulation(params_low);
        E_low_arr(trial) = E_low;
    end
    
    % Average sensitivity
    E_high = mean(E_high_arr);
    E_low = mean(E_low_arr);
    
    % Normalized sensitivity: (dY/Y) / (dX/X)
    Sensitivity.Energy(i) = ((E_high - E_low) / E_base) / (2 * perturbation);
    
    % Run single for other metrics
    rng(2026);
    params_high = BaseParams;
    params_high.(param_name) = base_val * (1 + perturbation);
    [~, P_avg_h, P_peak_h, Life_h, T_max_h] = run_simulation(params_high);
    
    params_low = BaseParams;
    params_low.(param_name) = base_val * (1 - perturbation);
    [~, P_avg_l, P_peak_l, Life_l, T_max_l] = run_simulation(params_low);
    
    Sensitivity.AvgPower(i) = ((P_avg_h - P_avg_l) / P_avg_base) / (2 * perturbation);
    Sensitivity.PeakPower(i) = ((P_peak_h - P_peak_l) / P_peak_base) / (2 * perturbation);
    Sensitivity.BatteryLife(i) = ((Life_h - Life_l) / Life_base) / (2 * perturbation);
    Sensitivity.MaxTemp(i) = ((T_max_h - T_max_l) / T_max_base) / (2 * perturbation);
    
    fprintf('  [%d/%d] %s: S_energy = %.4f\n', i, NumParams, param_name, Sensitivity.Energy(i));
end

%% ========================================================================
%  SECTION 3: MONTE CARLO UNCERTAINTY ANALYSIS
% ========================================================================

fprintf('\n----------------------------------------------------------\n');
fprintf('SECTION 3: Monte Carlo Uncertainty Analysis\n');
fprintf('----------------------------------------------------------\n\n');

N_MC = 500;  % Number of Monte Carlo samples
CV = 0.15;   % Coefficient of variation (15% uncertainty)

MC_Results.Energy = zeros(N_MC, 1);
MC_Results.AvgPower = zeros(N_MC, 1);
MC_Results.PeakPower = zeros(N_MC, 1);
MC_Results.BatteryLife = zeros(N_MC, 1);
MC_Results.MaxTemp = zeros(N_MC, 1);

fprintf('Running %d Monte Carlo simulations...\n', N_MC);

for mc = 1:N_MC
    if mod(mc, 50) == 0
        fprintf('  Progress: %d/%d (%.0f%%)\n', mc, N_MC, mc/N_MC*100);
    end
    
    rng(1000 + mc);
    
    % Generate random parameters with normal distribution
    params_mc = BaseParams;
    for i = 1:NumParams
        param_name = ParamNames{i};
        base_val = BaseParams.(param_name);
        % Log-normal to ensure positivity
        params_mc.(param_name) = base_val * exp(CV * randn());
    end
    
    [E, P_avg, P_peak, Life, T_max] = run_simulation(params_mc);
    
    MC_Results.Energy(mc) = E;
    MC_Results.AvgPower(mc) = P_avg;
    MC_Results.PeakPower(mc) = P_peak;
    MC_Results.BatteryLife(mc) = Life;
    MC_Results.MaxTemp(mc) = T_max;
end

% Statistics
MC_Stats.Energy.mean = mean(MC_Results.Energy);
MC_Stats.Energy.std = std(MC_Results.Energy);
MC_Stats.Energy.cv = MC_Stats.Energy.std / MC_Stats.Energy.mean;
MC_Stats.Energy.ci95 = [prctile(MC_Results.Energy, 2.5), prctile(MC_Results.Energy, 97.5)];

MC_Stats.AvgPower.mean = mean(MC_Results.AvgPower);
MC_Stats.AvgPower.std = std(MC_Results.AvgPower);
MC_Stats.AvgPower.cv = MC_Stats.AvgPower.std / MC_Stats.AvgPower.mean;
MC_Stats.AvgPower.ci95 = [prctile(MC_Results.AvgPower, 2.5), prctile(MC_Results.AvgPower, 97.5)];

MC_Stats.BatteryLife.mean = mean(MC_Results.BatteryLife);
MC_Stats.BatteryLife.std = std(MC_Results.BatteryLife);
MC_Stats.BatteryLife.cv = MC_Stats.BatteryLife.std / MC_Stats.BatteryLife.mean;
MC_Stats.BatteryLife.ci95 = [prctile(MC_Results.BatteryLife, 2.5), prctile(MC_Results.BatteryLife, 97.5)];

fprintf('\nMonte Carlo Results (N=%d, CV=%.0f%%):\n', N_MC, CV*100);
fprintf('  Energy Consumption:\n');
fprintf('    Mean: %.2f Wh, Std: %.2f Wh, CV: %.1f%%\n', ...
    MC_Stats.Energy.mean, MC_Stats.Energy.std, MC_Stats.Energy.cv*100);
fprintf('    95%% CI: [%.2f, %.2f] Wh\n', MC_Stats.Energy.ci95(1), MC_Stats.Energy.ci95(2));
fprintf('  Average Power:\n');
fprintf('    Mean: %.1f mW, Std: %.1f mW, CV: %.1f%%\n', ...
    MC_Stats.AvgPower.mean, MC_Stats.AvgPower.std, MC_Stats.AvgPower.cv*100);
fprintf('  Battery Life:\n');
fprintf('    Mean: %.1f h, Std: %.1f h, CV: %.1f%%\n', ...
    MC_Stats.BatteryLife.mean, MC_Stats.BatteryLife.std, MC_Stats.BatteryLife.cv*100);
fprintf('    95%% CI: [%.1f, %.1f] hours\n', MC_Stats.BatteryLife.ci95(1), MC_Stats.BatteryLife.ci95(2));

%% ========================================================================
%  SECTION 4: SOBOL GLOBAL SENSITIVITY ANALYSIS
% ========================================================================

fprintf('\n----------------------------------------------------------\n');
fprintf('SECTION 4: Sobol Global Sensitivity Analysis\n');
fprintf('----------------------------------------------------------\n\n');

% Select key parameters for Sobol analysis (most influential ones)
KeyParams = {'Batt_Q_design', 'PMIC_eff', 'SoC_C_eff', 'Disp_Beta_panel', ...
             'Conn_5G_active_high', 'Conn_WiFi_active', 'Therm_R_th'};
NumKeyParams = length(KeyParams);

% Sobol sampling (using Saltelli's method approximation)
N_Sobol = 256;  % Base sample size
Total_Samples = N_Sobol * (2 + NumKeyParams);

fprintf('Running Sobol analysis with %d key parameters...\n', NumKeyParams);
fprintf('Total evaluations: %d\n\n', Total_Samples);

% Generate Sobol matrices A, B, and AB_i
rng(42);

% Matrix A (N x k)
A_matrix = rand(N_Sobol, NumKeyParams);
% Matrix B (N x k)
B_matrix = rand(N_Sobol, NumKeyParams);

% Convert to parameter space (uniform +/- 30%)
param_range = 0.30;

% Evaluate model on A
Y_A = zeros(N_Sobol, 1);
for n = 1:N_Sobol
    params = convert_sobol_to_params(A_matrix(n,:), KeyParams, BaseParams, param_range);
    rng(5000 + n);
    [Y_A(n), ~, ~, ~, ~] = run_simulation(params);
end

% Evaluate model on B
Y_B = zeros(N_Sobol, 1);
for n = 1:N_Sobol
    params = convert_sobol_to_params(B_matrix(n,:), KeyParams, BaseParams, param_range);
    rng(5000 + n);
    [Y_B(n), ~, ~, ~, ~] = run_simulation(params);
end

% Evaluate model on AB_i matrices
Y_ABi = zeros(N_Sobol, NumKeyParams);
Y_BAi = zeros(N_Sobol, NumKeyParams);

for i = 1:NumKeyParams
    fprintf('  Processing parameter %d/%d: %s\n', i, NumKeyParams, KeyParams{i});
    
    for n = 1:N_Sobol
        % AB_i: A with i-th column from B
        AB_i = A_matrix(n,:);
        AB_i(i) = B_matrix(n, i);
        params = convert_sobol_to_params(AB_i, KeyParams, BaseParams, param_range);
        rng(5000 + n);
        [Y_ABi(n,i), ~, ~, ~, ~] = run_simulation(params);
        
        % BA_i: B with i-th column from A
        BA_i = B_matrix(n,:);
        BA_i(i) = A_matrix(n, i);
        params = convert_sobol_to_params(BA_i, KeyParams, BaseParams, param_range);
        rng(5000 + n);
        [Y_BAi(n,i), ~, ~, ~, ~] = run_simulation(params);
    end
end

% Calculate Sobol indices
f0 = mean([Y_A; Y_B]);
VarY = var([Y_A; Y_B]);

Sobol.S1 = zeros(NumKeyParams, 1);  % First-order
Sobol.ST = zeros(NumKeyParams, 1);  % Total-order

for i = 1:NumKeyParams
    % First-order (Jansen estimator)
    Sobol.S1(i) = (mean(Y_B .* Y_ABi(:,i)) - f0^2) / VarY;
    
    % Total-order
    Sobol.ST(i) = 0.5 * mean((Y_A - Y_ABi(:,i)).^2) / VarY;
end

% Ensure bounds
Sobol.S1 = max(0, min(1, Sobol.S1));
Sobol.ST = max(0, min(1, Sobol.ST));

fprintf('\nSobol Sensitivity Indices:\n');
fprintf('%-25s  S1 (First)  ST (Total)  Interaction\n', 'Parameter');
fprintf('------------------------------------------------------------\n');
for i = 1:NumKeyParams
    interaction = max(0, Sobol.ST(i) - Sobol.S1(i));
    fprintf('%-25s   %.4f       %.4f       %.4f\n', KeyParams{i}, ...
        Sobol.S1(i), Sobol.ST(i), interaction);
end
fprintf('------------------------------------------------------------\n');
fprintf('Sum of S1: %.4f (should be ~1.0 if no interactions)\n', sum(Sobol.S1));

%% ========================================================================
%  SECTION 5: USAGE PATTERN SENSITIVITY
% ========================================================================

fprintf('\n----------------------------------------------------------\n');
fprintf('SECTION 5: Usage Pattern Sensitivity Analysis\n');
fprintf('----------------------------------------------------------\n\n');

% Define different user profiles
UserProfiles.LightUser = [0.50, 0.35, 0.10, 0.05];    % Mostly sleep/light
UserProfiles.AverageUser = [0.35, 0.35, 0.15, 0.15];  % Balanced
UserProfiles.HeavyGamer = [0.20, 0.20, 0.15, 0.45];   % Gaming focus
UserProfiles.Streamer = [0.25, 0.25, 0.40, 0.10];     % Video streaming
UserProfiles.Business = [0.30, 0.50, 0.10, 0.10];     % Messaging/email

ProfileNames = fieldnames(UserProfiles);
NumProfiles = length(ProfileNames);

UsageResults.Profile = cell(NumProfiles, 1);
UsageResults.Energy_Wh = zeros(NumProfiles, 1);
UsageResults.AvgPower_mW = zeros(NumProfiles, 1);
UsageResults.PeakPower_mW = zeros(NumProfiles, 1);
UsageResults.BatteryLife_h = zeros(NumProfiles, 1);
UsageResults.MaxTemp_C = zeros(NumProfiles, 1);

fprintf('Evaluating %d user profiles...\n\n', NumProfiles);

for p = 1:NumProfiles
    profile_name = ProfileNames{p};
    profile = UserProfiles.(profile_name);
    
    rng(2026);
    [E, P_avg, P_peak, Life, T_max] = run_simulation(BaseParams, profile);
    
    UsageResults.Profile{p} = profile_name;
    UsageResults.Energy_Wh(p) = E;
    UsageResults.AvgPower_mW(p) = P_avg;
    UsageResults.PeakPower_mW(p) = P_peak;
    UsageResults.BatteryLife_h(p) = Life;
    UsageResults.MaxTemp_C(p) = T_max;
    
    fprintf('  %s:\n', profile_name);
    fprintf('    Profile: [Sleep=%.0f%%, Light=%.0f%%, Stream=%.0f%%, Game=%.0f%%]\n', ...
        profile(1)*100, profile(2)*100, profile(3)*100, profile(4)*100);
    fprintf('    Energy: %.2f Wh, Battery Life: %.1f h, Max Temp: %.1f C\n\n', E, Life, T_max);
end

% Usage sensitivity (how much does shifting usage affect outcomes?)
fprintf('Usage Pattern Sensitivity:\n');
fprintf('  Battery Life Range: %.1f - %.1f hours (%.0f%% variation)\n', ...
    min(UsageResults.BatteryLife_h), max(UsageResults.BatteryLife_h), ...
    (max(UsageResults.BatteryLife_h) - min(UsageResults.BatteryLife_h)) / mean(UsageResults.BatteryLife_h) * 100);
fprintf('  Energy Range: %.2f - %.2f Wh (%.0f%% variation)\n', ...
    min(UsageResults.Energy_Wh), max(UsageResults.Energy_Wh), ...
    (max(UsageResults.Energy_Wh) - min(UsageResults.Energy_Wh)) / mean(UsageResults.Energy_Wh) * 100);

%% ========================================================================
%  SECTION 6: ROBUSTNESS ANALYSIS
% ========================================================================

fprintf('\n----------------------------------------------------------\n');
fprintf('SECTION 6: Model Robustness Analysis\n');
fprintf('----------------------------------------------------------\n\n');

% Test robustness by increasing uncertainty levels
CV_levels = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30];
N_robust = 200;

Robustness.CV = CV_levels;
Robustness.Energy_CV = zeros(length(CV_levels), 1);
Robustness.BattLife_CV = zeros(length(CV_levels), 1);

fprintf('Testing robustness across uncertainty levels...\n');

for cv_idx = 1:length(CV_levels)
    cv = CV_levels(cv_idx);
    
    E_samples = zeros(N_robust, 1);
    Life_samples = zeros(N_robust, 1);
    
    for n = 1:N_robust
        rng(8000 + n);
        
        params_r = BaseParams;
        for i = 1:NumParams
            param_name = ParamNames{i};
            base_val = BaseParams.(param_name);
            params_r.(param_name) = base_val * exp(cv * randn());
        end
        
        [E, ~, ~, Life, ~] = run_simulation(params_r);
        E_samples(n) = E;
        Life_samples(n) = Life;
    end
    
    Robustness.Energy_CV(cv_idx) = std(E_samples) / mean(E_samples);
    Robustness.BattLife_CV(cv_idx) = std(Life_samples) / mean(Life_samples);
    
    fprintf('  CV_input = %.0f%%: Energy_CV = %.1f%%, BattLife_CV = %.1f%%\n', ...
        cv*100, Robustness.Energy_CV(cv_idx)*100, Robustness.BattLife_CV(cv_idx)*100);
end

% Robustness metric: ratio of output uncertainty to input uncertainty
fprintf('\nRobustness Metrics (Output CV / Input CV):\n');
for cv_idx = 1:length(CV_levels)
    cv = CV_levels(cv_idx);
    robustness_E = Robustness.Energy_CV(cv_idx) / cv;
    robustness_L = Robustness.BattLife_CV(cv_idx) / cv;
    fprintf('  Input CV = %.0f%%: Energy amplification = %.2f, Battery Life amplification = %.2f\n', ...
        cv*100, robustness_E, robustness_L);
end

%% ========================================================================
%  SECTION 7: VISUALIZATION
% ========================================================================

fprintf('\n----------------------------------------------------------\n');
fprintf('SECTION 7: Generating Visualizations\n');
fprintf('----------------------------------------------------------\n\n');

% --- Figure 1: Local Sensitivity Tornado Chart ---
figure('Name', 'Local Sensitivity Analysis', 'Color', 'w', 'Position', [50, 400, 800, 500]);

[sorted_sens, sort_idx] = sort(abs(Sensitivity.Energy), 'descend');
top_n = min(12, NumParams);
sorted_names = ParamNames(sort_idx(1:top_n));
sorted_values = Sensitivity.Energy(sort_idx(1:top_n));

barh(1:top_n, sorted_values(end:-1:1), 'FaceColor', [0.2 0.5 0.8]);
set(gca, 'YTickLabel', sorted_names(end:-1:1));
xlabel('Normalized Sensitivity Index');
title('Local Sensitivity Analysis - Energy Consumption');
grid on;
xline(0, 'k--', 'LineWidth', 1.5);

% Add value labels
for i = 1:top_n
    val = sorted_values(top_n - i + 1);
    if val >= 0
        text(val + 0.02, i, sprintf('%.3f', val), 'VerticalAlignment', 'middle');
    else
        text(val - 0.02, i, sprintf('%.3f', val), 'VerticalAlignment', 'middle', 'HorizontalAlignment', 'right');
    end
end

% --- Figure 2: Monte Carlo Histograms ---
figure('Name', 'Monte Carlo Uncertainty', 'Color', 'w', 'Position', [50, 50, 1000, 400]);

subplot(1,3,1);
histogram(MC_Results.Energy, 30, 'FaceColor', [0.3 0.6 0.9], 'EdgeColor', 'w');
hold on;
xline(E_base, 'r--', 'LineWidth', 2);
xline(MC_Stats.Energy.ci95(1), 'k:', 'LineWidth', 1.5);
xline(MC_Stats.Energy.ci95(2), 'k:', 'LineWidth', 1.5);
xlabel('Total Energy (Wh)');
ylabel('Frequency');
title(sprintf('Energy (CV=%.1f%%)', MC_Stats.Energy.cv*100));
legend('Distribution', 'Baseline', '95% CI');

subplot(1,3,2);
histogram(MC_Results.AvgPower, 30, 'FaceColor', [0.9 0.5 0.2], 'EdgeColor', 'w');
hold on;
xline(P_avg_base, 'r--', 'LineWidth', 2);
xlabel('Average Power (mW)');
ylabel('Frequency');
title(sprintf('Avg Power (CV=%.1f%%)', MC_Stats.AvgPower.cv*100));

subplot(1,3,3);
histogram(MC_Results.BatteryLife, 30, 'FaceColor', [0.2 0.7 0.4], 'EdgeColor', 'w');
hold on;
xline(Life_base, 'r--', 'LineWidth', 2);
xline(MC_Stats.BatteryLife.ci95(1), 'k:', 'LineWidth', 1.5);
xline(MC_Stats.BatteryLife.ci95(2), 'k:', 'LineWidth', 1.5);
xlabel('Battery Life (hours)');
ylabel('Frequency');
title(sprintf('Battery Life (CV=%.1f%%)', MC_Stats.BatteryLife.cv*100));

% --- Figure 3: Sobol Sensitivity Indices ---
figure('Name', 'Sobol Global Sensitivity', 'Color', 'w', 'Position', [870, 400, 600, 450]);

bar_data = [Sobol.S1, Sobol.ST - Sobol.S1];
b = barh(1:NumKeyParams, bar_data, 'stacked');
b(1).FaceColor = [0.2 0.6 0.8];
b(2).FaceColor = [0.8 0.4 0.2];

set(gca, 'YTickLabel', KeyParams);
xlabel('Sobol Index');
title('Global Sensitivity Analysis (Sobol Indices)');
legend({'First-Order (S1)', 'Interaction (ST-S1)'}, 'Location', 'SouthEast');
grid on;
xlim([0 1]);

% --- Figure 4: Usage Profile Comparison ---
figure('Name', 'Usage Pattern Analysis', 'Color', 'w', 'Position', [870, 50, 600, 300]);

bar(categorical(UsageResults.Profile), UsageResults.BatteryLife_h, 'FaceColor', [0.4 0.7 0.3]);
ylabel('Battery Life (hours)');
title('Battery Life by User Profile');
grid on;

% Add value labels on bars
for i = 1:NumProfiles
    text(i, UsageResults.BatteryLife_h(i) + 0.5, sprintf('%.1fh', UsageResults.BatteryLife_h(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

% --- Figure 5: Robustness Analysis ---
figure('Name', 'Robustness Analysis', 'Color', 'w', 'Position', [100, 200, 600, 400]);

plot(CV_levels*100, Robustness.Energy_CV*100, 'b-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
plot(CV_levels*100, Robustness.BattLife_CV*100, 'r-s', 'LineWidth', 2, 'MarkerSize', 8);
plot(CV_levels*100, CV_levels*100, 'k--', 'LineWidth', 1);  % Reference line

xlabel('Input Parameter CV (%)');
ylabel('Output CV (%)');
title('Model Robustness: Output Uncertainty vs Input Uncertainty');
legend({'Energy', 'Battery Life', 'Linear (1:1)'}, 'Location', 'NorthWest');
grid on;

text(25, 15, 'Below line = Robust', 'FontSize', 10, 'Color', [0 0.5 0]);
text(10, 25, 'Above line = Sensitive', 'FontSize', 10, 'Color', [0.7 0 0]);

% --- Figure 6: Parameter Sensitivity Heatmap ---
figure('Name', 'Multi-Output Sensitivity', 'Color', 'w', 'Position', [720, 200, 700, 500]);

% Create sensitivity matrix for multiple outputs
SensMatrix = [Sensitivity.Energy, Sensitivity.AvgPower, Sensitivity.BatteryLife, Sensitivity.MaxTemp];
OutputNames = {'Energy', 'Avg Power', 'Battery Life', 'Max Temp'};

% Select top 10 parameters
[~, top_idx] = sort(max(abs(SensMatrix), [], 2), 'descend');
top_params = top_idx(1:min(10, NumParams));

imagesc(SensMatrix(top_params, :));
colorbar;
colormap(redblue(256));
caxis([-1 1]);

set(gca, 'XTick', 1:4, 'XTickLabel', OutputNames);
set(gca, 'YTick', 1:length(top_params), 'YTickLabel', ParamNames(top_params));
title('Parameter Sensitivity Heatmap');
xlabel('Output Metric');
ylabel('Parameter');

%% ========================================================================
%  SECTION 8: SUMMARY REPORT
% ========================================================================

fprintf('\n=========================================================\n');
fprintf('           SENSITIVITY ANALYSIS SUMMARY REPORT\n');
fprintf('=========================================================\n\n');

fprintf('1. MOST INFLUENTIAL PARAMETERS (Local Sensitivity):\n');
for i = 1:min(5, NumParams)
    fprintf('   %d. %s (S = %.4f)\n', i, ParamNames{sort_idx(i)}, abs(sorted_sens(i)));
end

fprintf('\n2. MONTE CARLO UNCERTAINTY BOUNDS (95%% Confidence):\n');
fprintf('   Energy: %.2f - %.2f Wh (baseline: %.2f Wh)\n', ...
    MC_Stats.Energy.ci95(1), MC_Stats.Energy.ci95(2), E_base);
fprintf('   Battery Life: %.1f - %.1f hours (baseline: %.1f h)\n', ...
    MC_Stats.BatteryLife.ci95(1), MC_Stats.BatteryLife.ci95(2), Life_base);

fprintf('\n3. GLOBAL SENSITIVITY (Sobol - Top Parameters):\n');
[~, sobol_sort] = sort(Sobol.ST, 'descend');
for i = 1:min(3, NumKeyParams)
    fprintf('   %d. %s (ST = %.4f, Interaction = %.4f)\n', ...
        i, KeyParams{sobol_sort(i)}, Sobol.ST(sobol_sort(i)), ...
        max(0, Sobol.ST(sobol_sort(i)) - Sobol.S1(sobol_sort(i))));
end

fprintf('\n4. USAGE PATTERN IMPACT:\n');
[~, max_idx] = max(UsageResults.BatteryLife_h);
[~, min_idx] = min(UsageResults.BatteryLife_h);
fprintf('   Best Battery Life: %s (%.1f hours)\n', ...
    UsageResults.Profile{max_idx}, UsageResults.BatteryLife_h(max_idx));
fprintf('   Worst Battery Life: %s (%.1f hours)\n', ...
    UsageResults.Profile{min_idx}, UsageResults.BatteryLife_h(min_idx));
fprintf('   Usage impact: %.0f%% variation in battery life\n', ...
    (max(UsageResults.BatteryLife_h) - min(UsageResults.BatteryLife_h)) / mean(UsageResults.BatteryLife_h) * 100);

fprintf('\n5. ROBUSTNESS ASSESSMENT:\n');
avg_amplification = mean([Robustness.Energy_CV; Robustness.BattLife_CV] ./ repmat(CV_levels', 2, 1), 'all');
fprintf('   Average uncertainty amplification: %.2fx\n', avg_amplification);
if avg_amplification < 1.0
    fprintf('   Assessment: Model is ROBUST (dampens input uncertainty)\n');
elseif avg_amplification < 1.5
    fprintf('   Assessment: Model has MODERATE sensitivity\n');
else
    fprintf('   Assessment: Model is SENSITIVE (amplifies input uncertainty)\n');
end

fprintf('\n=========================================================\n');
fprintf('                  Analysis Complete\n');
fprintf('=========================================================\n');

% Save results
save('sensitivity_results.mat', 'Sensitivity', 'Sobol', 'MC_Results', 'MC_Stats', ...
    'Robustness', 'UsageResults', 'BaseParams', 'ParamNames', 'KeyParams');
fprintf('\nResults saved to sensitivity_results.mat\n');

%% ========================================================================
%  LOCAL FUNCTIONS
% ========================================================================

function [total_energy, avg_power, peak_power, battery_life, temp_max] = ...
    run_simulation(params, usage_profile)
    % Simplified simulation for sensitivity analysis
    % Returns key metrics for 24-hour simulation
    
    T_hours = 24;
    dt = 60;  % 60 second steps for faster simulation
    steps = floor(T_hours * 3600 / dt);
    
    % Initialize
    Q_design = (params.Batt_Q_design / 1000) * 3600;  % As
    SOC_C = Q_design;
    Temp = 298.15;  % K
    
    total_energy = 0;
    power_history = zeros(1, steps);
    temp_history = zeros(1, steps);
    
    % Usage profile weights (Sleep, Light, Stream, Game)
    if nargin < 2
        usage_profile = [0.35, 0.35, 0.15, 0.15];  % Default
    end
    
    for t = 1:steps
        curr_hr = (t-1) * dt / 3600;
        
        % Determine state based on time and profile
        if curr_hr >= 23 || curr_hr < 7
            state = 1;  % Sleep
        else
            r = rand();
            cum_prob = cumsum(usage_profile);
            state = find(r <= cum_prob, 1);
            if isempty(state), state = 1; end
        end
        
        % Generate load based on state
        switch state
            case 1  % Sleep
                util = 1.5 + randn();
                freq = 0.3;
                bri = 0;
                P_conn = params.Conn_5G_idle;
            case 2  % Light
                util = 25 + 5*randn();
                freq = 1.2 + 0.2*randn();
                bri = 400 + 50*randn();
                P_conn = params.Conn_WiFi_active * 0.5 + params.Conn_5G_idle;
            case 3  % Stream
                util = 40 + 10*randn();
                freq = 1.6 + 0.3*randn();
                bri = 600 + 100*randn();
                P_conn = params.Conn_WiFi_active + params.Conn_BT_active;
            case 4  % Game
                util = 85 + 10*randn();
                freq = 2.6 + 0.3*randn();
                bri = 900 + 150*randn();
                P_conn = params.Conn_5G_active_high + params.Conn_BT_active;
                if rand() > 0.7
                    P_conn = P_conn + params.Conn_GPS;
                end
        end
        
        % Clamp values
        util = max(0, min(100, util));
        freq = max(0.2, min(3.2, freq));
        bri = max(0, min(params.Disp_L_max, bri));
        
        % Throttling
        if Temp > params.Therm_T_throttle
            throttle = max(0.5, 1.0 - (Temp - params.Therm_T_throttle)*0.15);
            freq = freq * throttle;
            bri = bri * throttle;
        end
        
        % Power calculations
        % Display power
        P_disp = params.Disp_P_static + params.Disp_P_dyn_slope * 60 + ...
                 params.Disp_Beta_panel * bri * 0.5;
        if state == 1, P_disp = 0; end
        
        % SoC power
        V_dd = params.SoC_V_min + (params.SoC_V_max - params.SoC_V_min) * (freq / 3.0);
        P_soc_dyn = params.SoC_C_eff * (freq*1e9) * V_dd^2 * (util/100);
        P_soc_leak = V_dd * params.SoC_I_leak * (Temp/298.15)^2;
        P_soc = P_soc_dyn + P_soc_leak;
        
        % Total power
        P_total = P_disp + P_soc + P_conn + 0.05;  % + base
        P_total = min(P_total, 8.0);  % Clamp
        
        % Battery discharge
        P_req = P_total / params.PMIC_eff;
        curr_soc_p = max(0.001, SOC_C / Q_design);
        V_ocv = 3.0 + 1.0*curr_soc_p - 0.4*exp(-15*curr_soc_p);
        R_int = params.Batt_R_internal * (1 + 0.5*exp(-10*curr_soc_p));
        
        delta = V_ocv^2 - 4 * R_int * P_req;
        if delta < 0
            I_batt = V_ocv/(2*R_int);
        else
            I_batt = (V_ocv - sqrt(delta))/(2*R_int);
        end
        
        % Night charging simulation
        if curr_hr < 7
            SOC_C = Q_design;
            P_total = 0.1;
            I_batt = 0;
        else
            SOC_C = SOC_C - I_batt * dt;
        end
        
        if SOC_C <= 0
            SOC_C = Q_design;  % Emergency recharge
        end
        
        % Thermal
        Heat = P_soc + I_batt^2*R_int + 0.5*P_disp + P_conn;
        dT = (Heat - (Temp - 298.15)/params.Therm_R_th) / params.Therm_C_th * dt;
        Temp = Temp + dT;
        
        % Record
        power_history(t) = P_total;
        temp_history(t) = Temp;
        total_energy = total_energy + P_total * dt / 3600;  % Wh
    end
    
    avg_power = mean(power_history) * 1000;  % mW
    peak_power = max(power_history) * 1000;  % mW
    temp_max = max(temp_history) - 273.15;   % Celsius
    
    % Battery life estimation (hours from 100% to 20%)
    active_energy = total_energy * (17/24);  % Excluding night charging
    battery_capacity_Wh = params.Batt_Q_design * params.Batt_V_nom / 1000;
    usable_capacity = battery_capacity_Wh * 0.8;  % 80% usable
    battery_life = usable_capacity / (active_energy / 17) * 0.9;  % hours
end

function params = convert_sobol_to_params(u_vec, KeyParams, BaseParams, param_range)
    params = BaseParams;
    for j = 1:length(KeyParams)
        base_val = BaseParams.(KeyParams{j});
        % Map uniform [0,1] to [1-range, 1+range]
        multiplier = (1 - param_range) + 2 * param_range * u_vec(j);
        params.(KeyParams{j}) = base_val * multiplier;
    end
end

function c = redblue(m)
    % Red-White-Blue colormap for diverging data
    if nargin < 1, m = 256; end
    
    % Create the colormap
    n = ceil(m/2);
    
    % Red to white
    r1 = linspace(0.7, 1, n)';
    g1 = linspace(0.2, 1, n)';
    b1 = linspace(0.2, 1, n)';
    
    % White to blue
    r2 = linspace(1, 0.2, n)';
    g2 = linspace(1, 0.4, n)';
    b2 = linspace(1, 0.8, n)';
    
    c = [r1, g1, b1; r2(2:end), g2(2:end), b2(2:end)];
    
    % Ensure correct size
    if size(c,1) > m
        c = c(1:m, :);
    end
end
