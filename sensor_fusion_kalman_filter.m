
% Sensor Fusion Step 3: Kalman Filter Implementation
clear; clc; close all;

%% Parameters
fs = 100;               % Sampling frequency (Hz)
T = 10;                 % Duration (seconds)
t = 0:1/fs:T;           % Time vector

% True angle (ground truth)
true_angle = 45 * sin(2 * pi * 0.2 * t);  % +/-45 degrees

%% Simulated Sensors
encoder_resolution = 1024;
quantization = 360 / encoder_resolution;
encoder_noise = 0.5 * randn(size(t));
encoder_reading = round((true_angle + encoder_noise) / quantization) * quantization;

gyro_bias = 0.2;
gyro_noise = 0.5 * randn(size(t));
gyro_rate = [0, diff(true_angle)*fs];
gyro_reading = gyro_rate + gyro_bias + gyro_noise;

accel_noise = 2 * randn(size(t));
accel_reading = true_angle + accel_noise;

%% Kalman Filter Initialization
x = [accel_reading(1); 0];    % Initial state [angle; bias]
P = eye(2);                   % Initial error covariance
Q = [0.01 0; 0 0.003];        % Process noise covariance
R = 4;                        % Measurement noise (accelerometer)
A = [1 -1/fs; 0 1];           % State transition matrix
B = [1/fs; 0];                % Control input model (gyro input)
H = [1 0];                    % Measurement model

fused_kalman = zeros(size(t));
fused_kalman(1) = x(1);

%% Kalman Filter Loop
for i = 2:length(t)
    % Predict
    x = A * x + B * gyro_reading(i);
    P = A * P * A' + Q;

    % Update
    K = P * H' / (H * P * H' + R);            % Kalman gain
    x = x + K * (accel_reading(i) - H * x);   % State update
    P = (eye(2) - K * H) * P;                 % Covariance update

    fused_kalman(i) = x(1);                   % Store estimated angle
end

%% Plot Results
figure('Name', 'Kalman Filter Sensor Fusion');
plot(t, true_angle, 'k', 'LineWidth', 1.5); hold on;
plot(t, accel_reading, 'g:');
plot(t, fused_kalman, 'm', 'LineWidth', 2);
legend('True Angle', 'Accelerometer', 'Kalman Filter Estimate');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Sensor Fusion using Kalman Filter');
grid on;
