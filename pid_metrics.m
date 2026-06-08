%% PID Performance Metrics
%% Extracts rise time, overshoot, settling time, MSE

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

fprintf('==============================================\n');
fprintf('  PID Temperature Controller - Metrics\n');
fprintf('==============================================\n\n');

%% -------------------------------------------------------
%  stepinfo() on temperature rise signal
%% -------------------------------------------------------
y_rise = T_actual - T_amb;   % Temperature rise above ambient
r_rise = r;                  % Reference rise = 50 degC

info = stepinfo(y_rise, t_cl, r_rise, ...
    'SettlingTimeThreshold', 0.02, ...
    'RiseTimeLimits',        [0.1, 0.9]);

MSE  = mean((T_set - T_actual).^2);
e_ss = T_set - T_actual(end);

%% -------------------------------------------------------
%  Print Table
%% -------------------------------------------------------
fprintf('PID Gains: Kp=%.2f  Ki=%.4f  Kd=%.2f\n\n', Kp, Ki, Kd);
fprintf('%-30s %-15s\n', 'Metric', 'Value');
fprintf('%s\n', repmat('-',1,46));
fprintf('%-30s %-15.3f\n', 'Rise Time (s)',            info.RiseTime);
fprintf('%-30s %-15.3f\n', 'Peak Time (s)',             info.PeakTime);
fprintf('%-30s %-15.3f\n', 'Overshoot (%)',             info.Overshoot);
fprintf('%-30s %-15.3f\n', 'Settling Time (s)',         info.SettlingTime);
fprintf('%-30s %-15.4f\n', 'Steady-State Error (degC)', e_ss);
fprintf('%-30s %-15.6f\n', 'MSE',                       MSE);
fprintf('%s\n\n', repmat('-',1,46));

%% -------------------------------------------------------
%  Annotated Plot
%% -------------------------------------------------------
figure('Name','PID Metrics','Color','w','Position',[50 50 1600 900]);

plot(t_cl, T_ref,    'r--', 'LineWidth', 1.5); hold on;
plot(t_cl, T_actual, 'b-',  'LineWidth', 2.5);

yline(T_set * 1.02, 'g:', '+2%', ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment','left');

yline(T_set * 0.98, 'g:', '-2%', ...
    'LineWidth', 1.2, 'LabelHorizontalAlignment','left');

if ~isnan(info.RiseTime)
    xline(info.RiseTime, 'm--', ...
        sprintf('Rise\n%.1fs', info.RiseTime), ...
        'LineWidth', 1.2, ...
        'LabelVerticalAlignment','bottom');
end

if ~isnan(info.SettlingTime)
    xline(info.SettlingTime, 'k--', ...
        sprintf('Settle\n%.1fs', info.SettlingTime), ...
        'LineWidth', 1.2, ...
        'LabelVerticalAlignment','bottom');
end

if info.Overshoot > 0
    [pk, pi_] = max(T_actual);
    plot(t_cl(pi_), pk, 'rv', ...
        'MarkerSize', 10, ...
        'MarkerFaceColor','r');

    text(t_cl(pi_)+3, pk, ...
        sprintf('%.2f%%', info.Overshoot), ...
        'Color','r', ...
        'FontSize',12, ...
        'FontWeight','bold');
end

xlabel('Time (s)');
ylabel('Temperature (degC)');
title('PID Controller - Annotated Step Response (Setpoint: 75C)');
legend('Setpoint 75C','Actual T(t)','Location','southeast');
grid on; box on;

metrics_str = sprintf(['Kp=%.1f  Ki=%.4f  Kd=%.1f\n' ...
    'Rise Time: %.1f s\n' ...
    'Overshoot: %.2f%%\n' ...
    'Settling: %.1f s\n' ...
    'MSE: %.4f'], ...
    Kp, Ki, Kd, info.RiseTime, info.Overshoot, info.SettlingTime, MSE);

annotation('textbox',[0.62 0.15 0.28 0.28], ...
    'String',metrics_str, ...
    'FitBoxToText','on', ...
    'BackgroundColor',[1 1 0.8], ...
    'EdgeColor','k', ...
    'FontSize',12, ...
    'FontWeight','bold');

exportgraphics(gcf, 'pid_metrics_plot.png', 'Resolution', 600);
fprintf('Figure saved: pid_metrics_plot.png\n\n');

%% -------------------------------------------------------
%  Save for comparison
%% -------------------------------------------------------
pid_data.Kp           = Kp;
pid_data.Ki           = Ki;
pid_data.Kd           = Kd;
pid_data.RiseTime     = info.RiseTime;
pid_data.PeakTime     = info.PeakTime;
pid_data.Overshoot    = info.Overshoot;
pid_data.SettlingTime = info.SettlingTime;
pid_data.SSError      = e_ss;
pid_data.MSE          = MSE;

save('pid_metrics.mat', 'pid_data');

fprintf('Saved: pid_metrics.mat\n');