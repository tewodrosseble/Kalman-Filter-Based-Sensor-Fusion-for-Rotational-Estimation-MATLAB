
% Step 4: Animate Rotating Pointer with Kalman Filter Output
clear; clc; close all;

%% Parameters
fs = 100;               % Sampling frequency (Hz)
T = 10;                 % Duration (seconds)
t = 0:1/fs:T;           % Time vector

% True angle (ground truth)
true_angle = 45 * sin(2 * pi * 0.2 * t);  % +/-45 degrees

%% Simulated Sensors
gyro_bias = 0.2;
gyro_noise = 0.5 * randn(size(t));
gyro_rate = [0, diff(true_angle)*fs];
gyro_reading = gyro_rate + gyro_bias + gyro_noise;

accel_noise = 2 * randn(size(t));
accel_reading = true_angle + accel_noise;

%% Kalman Filter Initialization
x = [accel_reading(1); 0];
P = eye(2);
Q = [0.01 0; 0 0.003];
R = 4;
A = [1 -1/fs; 0 1];
B = [1/fs; 0];
H = [1 0];

fused_kalman = zeros(size(t));
fused_kalman(1) = x(1);

%% Kalman Filter Loop
for i = 2:length(t)
    x = A * x + B * gyro_reading(i);
    P = A * P * A' + Q;

    K = P * H' / (H * P * H' + R);
    x = x + K * (accel_reading(i) - H * x);
    P = (eye(2) - K * H) * P;

    fused_kalman(i) = x(1);
end

%% Animation
figure('Name', 'Rotating Pointer – Kalman Filter Estimate');
r = 1; % length of pointer

for k = 1:5:length(t)  % skip frames for smoother animation
    cla;
    hold on;
    % Draw circle
    theta_disk = linspace(0, 2*pi, 100);
    plot(cos(theta_disk), sin(theta_disk), 'k:');

    % Draw pointer (use fused_kalman angle)
    angle_rad = deg2rad(fused_kalman(k));
    x = [0 r * cos(angle_rad)];
    y = [0 r * sin(angle_rad)];
    plot(x, y, 'r', 'LineWidth', 3);

    axis equal;
    axis([-1.2 1.2 -1.2 1.2]);
    title(sprintf('Estimated Angle: %.2f°', fused_kalman(k)));
    grid on;
    drawnow;
end
