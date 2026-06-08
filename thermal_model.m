%% Thermal System Plant Model
%% Temperature Control - System Modelling
%% Control Objective: Regulation to 75 degrees C

clc; clear; close all;

%% -------------------------------------------------------
%  Figure & Font Settings (Report Quality)
%% -------------------------------------------------------
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.2);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.1);
set(groot,'defaultAxesFontWeight','bold');

%% -------------------------------------------------------
%  Thermal System Parameters
%  First-order model: tau*dT/dt + T = K*Q + T_amb
%% -------------------------------------------------------
K     = 0.5;    % Steady-state gain (degC per % heater power)
tau   = 50;     % Thermal time constant (seconds)
T_amb = 25;     % Ambient temperature (degC)
T_set = 75;     % Setpoint temperature (degC)

fprintf('Thermal System Parameters\n');
fprintf('K   = %.2f  degC per percent power\n', K);
fprintf('tau = %.1f  seconds\n', tau);
fprintf('T_ambient = %.1f degC\n', T_amb);
fprintf('Setpoint  = %.1f degC\n\n', T_set);

%% -------------------------------------------------------
%  Transfer Function: G(s) = K / (tau*s + 1)
%  Input : heater power Q (percent)
%  Output: temperature rise above ambient (degC)
%% -------------------------------------------------------
num = [K];
den = [tau, 1];
G   = tf(num, den);

fprintf('Transfer Function G(s) = K / (tau*s + 1)\n');
G

%% -------------------------------------------------------
%  State-Space Representation
%  State : x = T - T_amb  (temperature rise)
%  Input : u = Q (heater power, percent)
%  Output: y = T (absolute temperature)
%% -------------------------------------------------------
A_ss = -1/tau;
B_ss =  K/tau;
C_ss =  1;
D_ss =  0;
sys_ss = ss(A_ss, B_ss, C_ss, D_ss);

fprintf('State-Space: dT/dt = (%.4f)*T + (%.4f)*Q\n\n', A_ss, B_ss);

%% -------------------------------------------------------
%  System Analysis
%% -------------------------------------------------------
fprintf('Pole location : %.4f\n',   pole(G));
fprintf('DC Gain       : %.4f\n',   dcgain(G));
fprintf('Time Constant : %.2f s\n', tau);
fprintf('Settling Time : ~%.1f s (4*tau)\n\n', 4*tau);

%% -------------------------------------------------------
%  Open-Loop Step Response
%  Apply 100 percent heater power, observe temperature rise
%% -------------------------------------------------------
t_sim = 0:0.1:400;
[y_ol, t_ol] = step(100 * G, t_sim);
T_ol = y_ol + T_amb;

figure('Name','Thermal Plant Open-Loop Analysis','Color','w',...
       'Position',[50 50 1800 1200]);

subplot(2,2,1);
plot(t_ol, T_ol, 'b-', 'LineWidth', 2); hold on;
yline(T_amb + 100*K, 'r--', 'Steady State', 'LineWidth', 1.2);
yline(T_amb, 'k:', 'Ambient 25C', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Temperature (degC)');
title('Open-Loop Step Response (100% Heater Power)');
grid on; box on;

subplot(2,2,2);
bode(G);
title('Bode Plot');
grid on;

subplot(2,2,3);
pzmap(G);
title('Pole-Zero Map');
grid on;

subplot(2,2,4);
t_imp = 0:0.1:300;
[y_imp, t_imp2] = impulse(G, t_imp);
plot(t_imp2, y_imp, 'm-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Temperature response');
title('Impulse Response');
grid on; box on;

sgtitle('Thermal System - Open-Loop Analysis', ...
    'FontSize', 18, 'FontWeight', 'bold');

exportgraphics(gcf,'thermal_openloop.png','Resolution',600);

%% -------------------------------------------------------
%  Save model for subsequent scripts
%% -------------------------------------------------------
save('thermal_params.mat', 'K','tau','T_amb','T_set','G','sys_ss','A_ss','B_ss','C_ss','D_ss');
fprintf('Model saved to thermal_params.mat\n');