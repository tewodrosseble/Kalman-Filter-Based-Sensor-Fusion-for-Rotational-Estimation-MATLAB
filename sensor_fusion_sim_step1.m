
% Sensor Fusion Simulation: IMU + Encoder for Angle Estimation (Step 1)
clear; clc; close all;

%% Parameters
fs = 100;               % Sampling frequency (Hz)
T = 10;                 % Duration (seconds)
t = 0:1/fs:T;           % Time vector

% True angle signal (sine wave motion)
true_angle = 45 * sin(2 * pi * 0.2 * t);  % +/-45 degrees

%% Simulate Encoder
encoder_resolution = 1024;               % ticks per revolution
quantization = 360 / encoder_resolution; % degrees per tick
encoder_noise = 0.5 * randn(size(t));    % encoder electrical noise
encoder_reading = round((true_angle + encoder_noise) / quantization) * quantization;

%% Simulate Gyroscope
gyro_bias = 0.2;                          % degrees/sec constant drift
gyro_noise = 0.5 * randn(size(t));       % random noise
gyro_rate = [0, diff(true_angle)*fs];    % angular velocity (deg/sec)
gyro_reading = gyro_rate + gyro_bias + gyro_noise;

%% Simulate Accelerometer
accel_noise = 2 * randn(size(t));        % more noisy than encoder
accel_reading = true_angle + accel_noise;

%% Plot All Sensors vs True Angle
figure('Name', 'Sensor Simulation – IMU + Encoder');
subplot(3,1,1);
plot(t, true_angle, 'k', t, encoder_reading, 'b');
legend('True Angle', 'Encoder');
ylabel('Angle (deg)');
title('Encoder Signal');

subplot(3,1,2);
plot(t, gyro_rate, 'k', t, gyro_reading, 'r');
legend('True Angular Velocity', 'Gyroscope');
ylabel('Angular Rate (deg/s)');
title('Gyroscope Signal');

subplot(3,1,3);
plot(t, true_angle, 'k', t, accel_reading, 'g');
legend('True Angle', 'Accelerometer');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Accelerometer Signal');
