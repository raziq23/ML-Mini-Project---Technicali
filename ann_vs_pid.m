%% ANN vs PID - Head-to-Head Comparison
%% Compares all performance metrics side by side

clc; clear; close all;

load('thermal_params.mat');
load('pid_results.mat');
load('pid_metrics.mat');       % contains pid_data
load('pid_disturbance.mat');   % contains pid_dist_data
load('ann_results.mat');
load('ann_model.mat');
load('ann_training_data.mat');

%% -------------------------------------------------------
%  Figure & Font Settings
%% -------------------------------------------------------
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.2);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.1);
set(groot,'defaultAxesFontWeight','bold');

%% -------------------------------------------------------
%  Compute ANN Performance Metrics
%% -------------------------------------------------------
T_ref_ann = T_set * ones(size(t_ann));
err_ann   = T_ref_ann - T_sim;

y_rise_ann = T_sim - T_amb;
r_rise     = T_set - T_amb;

ann_info = stepinfo(y_rise_ann, t_ann, r_rise, ...
    'SettlingTimeThreshold', 0.02, ...
    'RiseTimeLimits',        [0.1, 0.9]);

MSE_ann  = mean(err_ann.^2);
e_ss_ann = T_set - T_sim(end);

%% -------------------------------------------------------
%  ANN Disturbance Test
%% -------------------------------------------------------
dt      = 0.1;
t_end   = 500;
t_dist  = 150;
dist_dT = 10;   % +10 degC disturbance
t_d     = 0:dt:t_end;
n       = length(t_d);

T_ann_dist    = zeros(1,n);
Q_ann_dist    = zeros(1,n);
T_ann_dist(1) = T_amb;

err_int  = 0;
prev_err = T_set - T_amb;

for k = 2:n

    % External disturbance added after t = 150 s
    disturbance = 0;
    if t_d(k) >= t_dist
        disturbance = dist_dT;
    end

    T_measured = T_ann_dist(k-1) + disturbance;

    err     = T_set - T_measured;
    err_int = err_int + err * dt;
    err_der = (err - prev_err) / dt;
    prev_err = err;

    ann_input   = [err; err_int; err_der];
    ann_input_n = mapminmax('apply', ann_input, psX);

    Q_raw_n = net(ann_input_n);
    Q_raw   = mapminmax('reverse', Q_raw_n, psY);

    Q_ann_dist(k) = max(0, min(100, Q_raw));

    dTdt = (-(T_ann_dist(k-1) - T_amb) + K * Q_ann_dist(k)) / tau;
    T_ann_dist(k) = T_ann_dist(k-1) + dTdt * dt;
end

%% -------------------------------------------------------
%  ANN Disturbance Metrics
%% -------------------------------------------------------
idx_after = t_d >= t_dist;
ann_max_dev = max(abs(T_ann_dist(idx_after) - T_set));
ann_MSE_dist = mean((T_set - T_ann_dist).^2);

band = 0.02 * T_set;
t_after = t_d(idx_after) - t_dist;
T_after = T_ann_dist(idx_after);

rec_idx = find(abs(T_after - T_set) <= band, 1, 'first');

if ~isempty(rec_idx)
    ann_recovery_time = t_after(rec_idx);
else
    ann_recovery_time = NaN;
end

%% -------------------------------------------------------
%  Print Comparison Table
%% -------------------------------------------------------
fprintf('============================================\n');
fprintf('  PID vs ANN - Performance Comparison\n');
fprintf('============================================\n\n');

fprintf('%-25s %-15s %-15s\n', 'Metric', 'PID', 'ANN');
fprintf('%s\n', repmat('-',1,56));

fprintf('%-25s %-15.3f %-15.3f\n', ...
    'Rise Time (s)', pid_data.RiseTime, ann_info.RiseTime);

fprintf('%-25s %-15.3f %-15.3f\n', ...
    'Overshoot (%)', pid_data.Overshoot, ann_info.Overshoot);

fprintf('%-25s %-15.3f %-15.3f\n', ...
    'Settling Time (s)', pid_data.SettlingTime, ann_info.SettlingTime);

fprintf('%-25s %-15.6f %-15.6f\n', ...
    'MSE', pid_data.MSE, MSE_ann);

fprintf('%-25s %-15.4f %-15.4f\n', ...
    'SS Error (degC)', pid_data.SSError, e_ss_ann);

fprintf('%-25s %-15.3f %-15.3f\n', ...
    'Dist. Max Dev (degC)', pid_dist_data.max_deviation, ann_max_dev);

fprintf('%-25s %-15.3f %-15.3f\n', ...
    'Dist. Recovery (s)', pid_dist_data.recovery_time, ann_recovery_time);

fprintf('%s\n\n', repmat('-',1,56));

%% -------------------------------------------------------
%  Comparison Plots
%% -------------------------------------------------------
figure('Name','ANN vs PID Comparison','Color','w',...
       'Position',[50 50 1800 1200]);

% --- Plot 1: Reference tracking ---
subplot(2,3,1);
plot(t_cl,  T_ref,    'r--', 'LineWidth',1.5); hold on;
plot(t_cl,  T_actual, 'b-',  'LineWidth',2);
plot(t_ann, T_sim,    'g-',  'LineWidth',2);
xlabel('Time (s)');
ylabel('Temperature (degC)');
title('Reference Tracking');
legend('Setpoint 75C','PID','ANN','Location','southeast');
grid on; box on;

% --- Plot 2: Error comparison ---
subplot(2,3,2);
plot(t_cl,  T_set - T_actual, 'b-', 'LineWidth',2); hold on;
plot(t_ann, err_ann,          'g-', 'LineWidth',2);
yline(0,'k--','LineWidth',1);
xlabel('Time (s)');
ylabel('Error (degC)');
title('Tracking Error Comparison');
legend('PID Error','ANN Error','Location','northeast');
grid on; box on;

% --- Plot 3: Bar chart - metrics ---
subplot(2,3,3);
metrics_names = {'Rise Time','Overshoot','Settling'};
pid_vals = [pid_data.RiseTime, pid_data.Overshoot, pid_data.SettlingTime];
ann_vals = [ann_info.RiseTime, ann_info.Overshoot, ann_info.SettlingTime];

bar([pid_vals; ann_vals]');
set(gca,'XTickLabel', metrics_names);
title('Performance Metrics Comparison');
legend('PID','ANN','Location','northeast');
ylabel('Value');
grid on; box on;

% --- Plot 4: MSE bar ---
subplot(2,3,4);
bar([pid_data.MSE, MSE_ann]);
set(gca,'XTickLabel',{'PID','ANN'});
ylabel('MSE');
title('Mean Squared Error');
grid on; box on;

% --- Plot 5: Disturbance comparison ---
subplot(2,3,5);
plot(pid_dist_data.t, T_set*ones(size(pid_dist_data.t)), ...
    'r--','LineWidth',1.5); hold on;

plot(pid_dist_data.t, pid_dist_data.T_with_dist, ...
    'b-', 'LineWidth',2);

plot(t_d, T_ann_dist, ...
    'g-', 'LineWidth',2);

xline(t_dist,'k--',sprintf('Disturbance t=%ds',t_dist), ...
    'LineWidth',1.2);

xlabel('Time (s)');
ylabel('Temperature (degC)');
title('Disturbance Rejection');
legend('Setpoint','PID','ANN','Location','southeast');
grid on; box on;

% --- Plot 6: Control signal comparison ---
subplot(2,3,6);
plot(t_cl,  u_clamped, 'b-', 'LineWidth',2); hold on;
plot(t_ann, Q_sim,     'g-', 'LineWidth',2);
yline(100,'r--','100%','LineWidth',1);
yline(0,'k--','0%','LineWidth',1);
xlabel('Time (s)');
ylabel('Heater Power (%)');
title('Control Signal: Heater Power');
legend('PID','ANN','Location','northeast');
grid on; box on;

sgtitle('PID vs ANN Controller - Full Comparison', ...
    'FontSize',18,'FontWeight','bold');

exportgraphics(gcf, 'ann_vs_pid_comparison.png', 'Resolution', 600);

fprintf('Figure saved: ann_vs_pid_comparison.png\n');

%% -------------------------------------------------------
%  Save all comparison data for report
%% -------------------------------------------------------
comparison.pid_rise          = pid_data.RiseTime;
comparison.pid_os            = pid_data.Overshoot;
comparison.pid_settle        = pid_data.SettlingTime;
comparison.pid_mse           = pid_data.MSE;
comparison.pid_ss_error      = pid_data.SSError;
comparison.pid_dist_max_dev  = pid_dist_data.max_deviation;
comparison.pid_dist_recovery = pid_dist_data.recovery_time;

comparison.ann_rise          = ann_info.RiseTime;
comparison.ann_os            = ann_info.Overshoot;
comparison.ann_settle        = ann_info.SettlingTime;
comparison.ann_mse           = MSE_ann;
comparison.ann_ss_error      = e_ss_ann;
comparison.ann_dist_max_dev  = ann_max_dev;
comparison.ann_dist_recovery = ann_recovery_time;

save('comparison_results.mat', 'comparison');

fprintf('Saved: comparison_results.mat\n');