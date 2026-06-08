%% PID Controller for Temperature Control
%% Setpoint: 75 degrees C, Ambient: 25 degrees C

clc; clear; close all;
load('thermal_params.mat');

%% -------------------------------------------------------
%  Figure & Font Settings
%% -------------------------------------------------------
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.2);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.1);
set(groot,'defaultAxesFontWeight','bold');

%% -------------------------------------------------------
%  PID Tuning using pidtune()
%% -------------------------------------------------------
opts   = pidtuneOptions('CrossoverFrequency', 0.05, 'PhaseMargin', 70);
C_auto = pidtune(G, 'PID', opts);

fprintf('Auto-Tuned PID Gains:\n');
fprintf('  Kp = %.4f\n', C_auto.Kp);
fprintf('  Ki = %.4f\n', C_auto.Ki);
fprintf('  Kd = %.4f\n\n', C_auto.Kd);

%% -------------------------------------------------------
%  Manual Gains
%% -------------------------------------------------------
Kp = 5.0;
Ki = 0.05;
Kd = 10.0;

fprintf('Manual PID Gains:\n');
fprintf('  Kp = %.2f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %.2f\n\n', Kd);

N = 20;   % Derivative filter coefficient
C_pid = pid(Kp, Ki, Kd, N);

%% -------------------------------------------------------
%  Closed-Loop System
%% -------------------------------------------------------
r    = T_set - T_amb;
T_cl = feedback(C_pid * G, 1);

p_cl = pole(T_cl);
fprintf('Closed-Loop Poles:\n'); 
disp(p_cl);

if all(real(p_cl) < 0)
    fprintf('System is STABLE.\n\n');
end

%% -------------------------------------------------------
%  Simulate Closed-Loop Step Response
%% -------------------------------------------------------
t_sim        = 0:0.1:500;
[y_cl, t_cl] = step(r * T_cl, t_sim);

T_actual = y_cl + T_amb;
T_ref    = T_set * ones(size(t_cl));
error_cl = T_set - T_actual;

%% -------------------------------------------------------
%  Clamp heater output to 0-100 percent
%  Compute control signal u(t) using PID equation
%% -------------------------------------------------------
dt = t_cl(2) - t_cl(1);

error_derivative = [0; diff(error_cl)/dt];
error_integral   = cumtrapz(t_cl, error_cl);

u_out = Kp*error_cl + Ki*error_integral + Kd*error_derivative;
u_clamped = max(0, min(100, u_out));

%% -------------------------------------------------------
%  Plots
%% -------------------------------------------------------
figure('Name','PID Temperature Control','Color','w',...
       'Position',[50 50 1800 1200]);

subplot(2,2,1);
plot(t_cl, T_ref,    'r--', 'LineWidth', 1.5); hold on;
plot(t_cl, T_actual, 'b-',  'LineWidth', 2);
yline(T_amb, 'k:', '25C Ambient', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Temperature (degC)');
title('PID: Reference vs Actual Temperature');
legend('Setpoint 75C','Actual T(t)','Location','southeast');
grid on; box on;

subplot(2,2,2);
plot(t_cl, error_cl, 'r-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Error (degC)');
title('Tracking Error vs Time');
grid on; box on;

subplot(2,2,3);
plot(t_cl, u_clamped, 'm-', 'LineWidth', 2);
yline(100, 'r--', '100% max', 'LineWidth', 1);
yline(0,   'b--', '0% min',   'LineWidth', 1);
xlabel('Time (s)');
ylabel('Heater Power (%)');
title('Control Signal: Heater Power');
grid on; box on;

subplot(2,2,4);
[y_auto, t_auto] = step(r * feedback(C_auto*G,1), t_sim);
T_auto = y_auto + T_amb;

plot(t_cl,   T_ref,    'r--', 'LineWidth', 1.5); hold on;
plot(t_auto, T_auto,   'g-',  'LineWidth', 2);
plot(t_cl,   T_actual, 'b-',  'LineWidth', 2);
xlabel('Time (s)');
ylabel('Temperature (degC)');
title('Auto-Tuned vs Manual PID');
legend('Setpoint','Auto-Tuned','Manual','Location','southeast');
grid on; box on;

sgtitle('PID Temperature Controller', ...
    'FontSize', 18, 'FontWeight', 'bold');

exportgraphics(gcf,'pid_temperature.png','Resolution',600);

%% -------------------------------------------------------
%  Save results
%% -------------------------------------------------------
save('pid_results.mat', ...
    'Kp','Ki','Kd','N','C_pid','T_cl', ...
    't_cl','T_actual','T_ref','error_cl','u_clamped','r');

fprintf('Saved: pid_results.mat\n');