%% PID Disturbance Rejection Test
%% Injects a step disturbance at t=150s to test robustness

clc; clear; close all;
load('thermal_params.mat');
load('pid_results.mat');

%% -------------------------------------------------------
%  Figure & Font Settings
%% -------------------------------------------------------
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.2);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.1);
set(groot,'defaultAxesFontWeight','bold');

t_sim    = 0:0.1:500;
t_dist   = 150;     % Disturbance at 150 seconds
dist_mag = 10;      % +10 degC equivalent disturbance

fprintf('Disturbance Test\n');
fprintf('Injected at t = %d s, magnitude = %d degC equivalent\n\n', t_dist, dist_mag);

%% -------------------------------------------------------
%  Build disturbance-to-output transfer function
%  Disturbance enters at plant input
%% -------------------------------------------------------
T_dist_tf = feedback(G, C_pid);

%% -------------------------------------------------------
%  Simulate reference response
%% -------------------------------------------------------
ref_signal  = r * ones(size(t_sim));
dist_signal = zeros(size(t_sim));
dist_signal(t_sim >= t_dist) = dist_mag / K;

[y_ref,  ~] = lsim(T_cl,       ref_signal,  t_sim);
[y_dist, ~] = lsim(T_dist_tf,  dist_signal, t_sim);

T_nodist = y_ref + T_amb;
T_total  = y_ref + y_dist + T_amb;
err_dist = T_set - T_total;

%% -------------------------------------------------------
%  Recovery metrics after disturbance
%% -------------------------------------------------------
idx_after = t_sim >= t_dist;
t_after   = t_sim(idx_after) - t_dist;
T_after   = T_total(idx_after);

band    = 0.02 * T_set;
rec_idx = find(abs(T_after - T_set) <= band, 1, 'first');

if ~isempty(rec_idx)
    recovery_time = t_after(rec_idx);
    fprintf('Recovery Time: %.2f s\n', recovery_time);
else
    recovery_time = NaN;
    fprintf('Did not fully recover within simulation window.\n');
end

max_dev = max(abs(T_total(idx_after) - T_set));
MSE_nd  = mean((T_set - T_nodist).^2);
MSE_d   = mean((T_set - T_total).^2);

fprintf('Max Deviation from Setpoint: %.4f degC\n', max_dev);
fprintf('MSE (no disturbance): %.6f\n', MSE_nd);
fprintf('MSE (with disturbance): %.6f\n\n', MSE_d);

%% -------------------------------------------------------
%  Plots
%% -------------------------------------------------------
figure('Name','PID Disturbance Test','Color','w',...
       'Position',[50 50 1800 1100]);

subplot(2,2,[1 2]);
plot(t_sim, T_set*ones(size(t_sim)), 'r--', 'LineWidth', 1.5); hold on;
plot(t_sim, T_nodist,                'b-',  'LineWidth', 2);
plot(t_sim, T_total,                 'm-',  'LineWidth', 2);

xline(t_dist, 'k--', ...
      sprintf('Disturbance +%d degC @ t=%ds', dist_mag, t_dist), ...
      'LineWidth', 1.5, ...
      'LabelVerticalAlignment','bottom');

yline(T_set*1.02, 'g:', '+2%', 'LineWidth', 1);
yline(T_set*0.98, 'g:', '-2%', 'LineWidth', 1);

xlabel('Time (s)');
ylabel('Temperature (degC)');
title('PID Disturbance Rejection Test');
legend('Setpoint 75C','No Disturbance','With Disturbance','Location','southeast');
grid on; box on;

subplot(2,2,3);
plot(t_sim, T_set - T_nodist, 'b-', 'LineWidth', 2); hold on;
plot(t_sim, err_dist,         'm-', 'LineWidth', 2);
xline(t_dist, 'k--', 'LineWidth', 1.2);
yline(0, 'k-', 'LineWidth', 0.8);
xlabel('Time (s)');
ylabel('Error (degC)');
title('Error: With vs Without Disturbance');
legend('No Disturbance','With Disturbance','Location','northeast');
grid on; box on;

subplot(2,2,4);
plot(t_sim, dist_mag*(t_sim >= t_dist), 'r-', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Disturbance (degC)');
title('Injected Disturbance Signal');
ylim([-1, dist_mag*1.5]);
grid on; box on;

sgtitle('PID - Disturbance Rejection', ...
    'FontSize', 18, 'FontWeight', 'bold');

exportgraphics(gcf, 'pid_disturbance.png', 'Resolution', 600);

%% -------------------------------------------------------
%  Save for comparison
%% -------------------------------------------------------
pid_dist_data.t             = t_sim;
pid_dist_data.T_nodist      = T_nodist;
pid_dist_data.T_with_dist   = T_total;
pid_dist_data.error_dist    = err_dist;
pid_dist_data.recovery_time = recovery_time;
pid_dist_data.max_deviation = max_dev;
pid_dist_data.MSE_nodist    = MSE_nd;
pid_dist_data.MSE_dist      = MSE_d;

save('pid_disturbance.mat', 'pid_dist_data');

fprintf('Saved: pid_disturbance.mat\n');