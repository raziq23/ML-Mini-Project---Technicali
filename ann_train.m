%% ANN Controller - Build, Train and Validate
%% Supervised Learning: ANN learns to imitate PID control behaviour
%% Inputs : [error, integral of error, derivative of error]
%% Output : heater power Q (0-100 percent)
%% Requires: Deep Learning Toolbox

clc; clear; close all;
load('thermal_params.mat');
load('ann_training_data.mat');

fprintf('=== ANN Controller Training ===\n\n');

%% -------------------------------------------------------
%  Figure & Font Settings
%% -------------------------------------------------------
set(groot,'defaultAxesFontSize',14);
set(groot,'defaultTextFontSize',14);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.2);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.1);
set(groot,'defaultAxesFontWeight','bold');

%% -------------------------------------------------------
%  Prepare Data
%  Neural Network Toolbox expects [features x samples]
%% -------------------------------------------------------
X = ann_inputs';     % [3 x N]
Y = ann_outputs';    % [1 x N]

fprintf('Input  size: %d features x %d samples\n', size(X,1), size(X,2));
fprintf('Output size: %d x %d\n\n', size(Y,1), size(Y,2));

%% -------------------------------------------------------
%  Normalize Data
%% -------------------------------------------------------
[Xn, psX] = mapminmax(X);
[Yn, psY] = mapminmax(Y);

%% -------------------------------------------------------
%  Build Feedforward Neural Network
%  Architecture: 3 -> 15 -> 10 -> 1
%% -------------------------------------------------------
hidden_layers = [15, 10];

net = feedforwardnet(hidden_layers, 'trainscg');

net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'tansig';
net.layers{3}.transferFcn = 'purelin';

net.trainParam.epochs     = 500;
net.trainParam.goal       = 1e-5;
net.trainParam.showWindow = true;

net.divideParam.trainRatio = 0.70;
net.divideParam.valRatio   = 0.15;
net.divideParam.testRatio  = 0.15;

fprintf('Network Architecture: 3 -> 15 -> 10 -> 1\n');
fprintf('Training algorithm: Scaled Conjugate Gradient\n');
fprintf('Epochs: %d\n\n', net.trainParam.epochs);

%% -------------------------------------------------------
%  Train ANN
%% -------------------------------------------------------
[net, tr] = train(net, Xn, Yn);

fprintf('\nTraining complete.\n');
fprintf('Best validation performance: %.6f at epoch %d\n\n', ...
        tr.best_vperf, tr.best_epoch);

%% -------------------------------------------------------
%  Evaluate ANN on All Data
%% -------------------------------------------------------
Y_pred_n = net(Xn);
Y_pred   = mapminmax('reverse', Y_pred_n, psY);

Y_pred = max(0, min(100, Y_pred));

residuals  = Y - Y_pred;
MSE_train  = mean(residuals.^2);
RMSE_train = sqrt(MSE_train);

R_corr = corrcoef(Y, Y_pred);
R2 = R_corr(1,2)^2;

fprintf('ANN Prediction Performance:\n');
fprintf('  MSE  = %.6f\n',  MSE_train);
fprintf('  RMSE = %.6f\n',  RMSE_train);
fprintf('  R^2  = %.6f\n\n', R2);

%% -------------------------------------------------------
%  Closed-Loop Simulation with ANN Controller
%% -------------------------------------------------------
load('pid_results.mat');

dt    = 0.1;
t_end = 500;
t_ann = 0:dt:t_end;
n     = length(t_ann);

T_sim = zeros(1, n);
Q_sim = zeros(1, n);

err_int  = 0;
prev_err = T_set - T_amb;

T_sim(1) = T_amb;

for k = 2:n

    err     = T_set - T_sim(k-1);
    err_int = err_int + err * dt;
    err_der = (err - prev_err) / dt;
    prev_err = err;

    ann_input   = [err; err_int; err_der];
    ann_input_n = mapminmax('apply', ann_input, psX);

    Q_raw_n = net(ann_input_n);
    Q_raw   = mapminmax('reverse', Q_raw_n, psY);

    Q_sim(k) = max(0, min(100, Q_raw));

    dTdt = (-(T_sim(k-1) - T_amb) + K * Q_sim(k)) / tau;
    T_sim(k) = T_sim(k-1) + dTdt * dt;
end

%% -------------------------------------------------------
%  Plot Results
%% -------------------------------------------------------
figure('Name','ANN Training Results','Color','w',...
       'Position',[50 50 1800 1200]);

subplot(2,3,1);
plot(Y, 'b.', 'MarkerSize', 3); hold on;
plot(Y_pred, 'r.', 'MarkerSize', 3);
xlabel('Sample');
ylabel('Heater Power (%)');
title('ANN Prediction vs Target');
legend('Target','Predicted','Location','northeast');
grid on; box on;

subplot(2,3,2);
scatter(Y, Y_pred, 3, 'b', 'filled');
hold on;
plot([0 100], [0 100], 'r--', 'LineWidth', 1.5);
xlabel('Target Q (%)');
ylabel('Predicted Q (%)');
title(sprintf('Regression Plot  R^2 = %.4f', R2));
grid on; box on;

subplot(2,3,3);
histogram(residuals, 50, 'FaceColor','g');
xlabel('Residual (%)');
ylabel('Count');
title('Prediction Residuals');
grid on; box on;

subplot(2,3,4);
plot(t_ann, T_set*ones(size(t_ann)), 'r--', 'LineWidth', 1.5); hold on;
plot(t_ann, T_sim,                   'b-',  'LineWidth', 2);
xlabel('Time (s)');
ylabel('Temperature (degC)');
title('ANN Controller: Closed-Loop Response');
legend('Setpoint 75C','ANN Response','Location','southeast');
grid on; box on;

subplot(2,3,5);
plot(t_ann, T_set - T_sim, 'r-', 'LineWidth', 2);
yline(0, 'k--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Error (degC)');
title('ANN Tracking Error');
grid on; box on;

subplot(2,3,6);
plot(t_ann, Q_sim, 'm-', 'LineWidth', 2);
yline(100, 'r--', '100%', 'LineWidth', 1);
yline(0,   'b--', '0%',   'LineWidth', 1);
xlabel('Time (s)');
ylabel('Heater Power (%)');
title('ANN Control Signal');
grid on; box on;

sgtitle('ANN Controller - Training & Closed-Loop Results', ...
    'FontSize', 18, 'FontWeight', 'bold');

exportgraphics(gcf, 'ann_training_results.png', 'Resolution', 600);

%% -------------------------------------------------------
%  Save ANN and Simulation Results
%% -------------------------------------------------------
save('ann_model.mat', ...
    'net', 'tr', 'psX', 'psY', ...
    'MSE_train', 'RMSE_train', 'R2');

save('ann_results.mat', ...
    't_ann', 'T_sim', 'Q_sim');

fprintf('Saved: ann_model.mat\n');
fprintf('Saved: ann_results.mat\n');3