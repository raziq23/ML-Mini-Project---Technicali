%% ANN Training Data Collection
%% Runs PID simulation and records input-output pairs for ANN training
%% ANN Inputs : [error, integral of error, derivative of error]
%% ANN Output : heater power Q (percent)

clc; clear; close all;
load('thermal_params.mat');
load('pid_results.mat');

fprintf('Collecting ANN training data from PID simulation...\n\n');

%% -------------------------------------------------------
%  Simulate multiple setpoint scenarios for richer training data
%  This improves ANN generalisation
%% -------------------------------------------------------
setpoints  = [40, 50, 60, 75, 85];   % Various target temperatures (degC)
dt         = 0.1;                     % Time step (s)
t_end      = 500;                     % Simulation duration (s)
t_vec      = 0:dt:t_end;
n_steps    = length(t_vec);

all_inputs  = [];   % [error, integral_error, derivative_error]
all_outputs = [];   % [heater power Q]

for sp = setpoints
    r_sp = sp - T_amb;   % Reference rise above ambient

    % Closed-loop simulation
    [y_sp, t_sp] = step(r_sp * T_cl, t_vec);
    T_sp         = y_sp + T_amb;
    err_sp       = sp - T_sp;

    % Compute PID terms manually
    err_int  = cumtrapz(t_sp, err_sp);          % Integral of error
    err_der  = gradient(err_sp, dt);            % Derivative of error

    % Reconstruct control signal Q = Kp*e + Ki*int(e) + Kd*de/dt
    Q_pid = Kp*err_sp + Ki*err_int + Kd*err_der;
    Q_pid = max(0, min(100, Q_pid));             % Clamp 0-100%

    % Stack into training arrays
    inputs  = [err_sp, err_int, err_der];
    outputs = Q_pid;

    all_inputs  = [all_inputs;  inputs];
    all_outputs = [all_outputs; outputs];

    fprintf('Collected %d samples for setpoint %d degC\n', n_steps, sp);
end

fprintf('\nTotal training samples: %d\n\n', size(all_inputs, 1));

%% -------------------------------------------------------
%  Inspect data distribution
%% -------------------------------------------------------
figure('Name','Training Data Overview','Color','w','Position',[100 100 1000 600]);

subplot(2,2,1);
histogram(all_inputs(:,1), 50, 'FaceColor','b');
xlabel('Error (degC)'); ylabel('Count');
title('Distribution: Error');
grid on;

subplot(2,2,2);
histogram(all_inputs(:,2), 50, 'FaceColor','g');
xlabel('Integral of Error'); ylabel('Count');
title('Distribution: Integral Error');
grid on;

subplot(2,2,3);
histogram(all_inputs(:,3), 50, 'FaceColor','r');
xlabel('Derivative of Error'); ylabel('Count');
title('Distribution: Derivative Error');
grid on;

subplot(2,2,4);
histogram(all_outputs, 50, 'FaceColor','m');
xlabel('Heater Power Q (%)'); ylabel('Count');
title('Distribution: Output (Heater Power)');
grid on;

sgtitle('ANN Training Data Distribution', 'FontSize', 13, 'FontWeight', 'bold');
saveas(gcf, 'ann_data_distribution.png');

%% -------------------------------------------------------
%  Save training data
%% -------------------------------------------------------
ann_inputs  = all_inputs;
ann_outputs = all_outputs;

save('ann_training_data.mat', 'ann_inputs', 'ann_outputs', 'setpoints', 'dt');
fprintf('Training data saved to ann_training_data.mat\n');
fprintf('  ann_inputs  : [%d x 3]  (error, integral, derivative)\n', size(ann_inputs,1));
fprintf('  ann_outputs : [%d x 1]  (heater power)\n', size(ann_outputs,1));
