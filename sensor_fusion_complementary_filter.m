
% Sensor Fusion Step 2: Complementary Filter
clear; clc; close all;

%% Parameters
fs = 100;               % Sampling frequency (Hz)
T = 10;                 % Duration (seconds)
t = 0:1/fs:T;           % Time vector

% True angle signal (sine wave motion)
true_angle = 45 * sin(2 * pi * 0.2 * t);  % +/-45 degrees

%% Simulate Encoder
encoder_resolution = 1024;
quantization = 360 / encoder_resolution;
encoder_noise = 0.5 * randn(size(t));
encoder_reading = round((true_angle + encoder_noise) / quantization) * quantization;

%% Simulate Gyroscope
gyro_bias = 0.2;
gyro_noise = 0.5 * randn(size(t));
gyro_rate = [0, diff(true_angle)*fs];
gyro_reading = gyro_rate + gyro_bias + gyro_noise;

%% Simulate Accelerometer
accel_noise = 2 * randn(size(t));
accel_reading = true_angle + accel_noise;

%% Complementary Filter Fusion
alpha = 0.98;  % Gyro weight (close to 1 means rely more on gyro)
fused_angle = zeros(size(t));
fused_angle(1) = accel_reading(1);  % initialize from accelerometer

for i = 2:length(t)
    gyro_angle = fused_angle(i-1) + gyro_reading(i) * (1/fs);  % integrate gyro
    fused_angle(i) = alpha * gyro_angle + (1 - alpha) * accel_reading(i);  % fuse with accel
end

%% Plot Results
figure('Name', 'Complementary Filter Sensor Fusion');
plot(t, true_angle, 'k', 'LineWidth', 1.5); hold on;
plot(t, encoder_reading, 'b:');
plot(t, accel_reading, 'g:');
plot(t, fused_angle, 'r', 'LineWidth', 2);
legend('True Angle', 'Encoder', 'Accelerometer', 'Fused Angle');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Sensor Fusion using Complementary Filter');
grid on;
