
% Full GUI for Sensor Fusion with Kalman Filter and Pointer Animation
clear; clc; close all;

%% Parameters
fs = 100;
T = 10;
t = 0:1/fs:T;

% Ground truth angle
true_angle = 45 * sin(2 * pi * 0.2 * t);

% Simulate gyro
gyro_bias = 0.2;
gyro_noise = 0.5 * randn(size(t));
gyro_rate = [0, diff(true_angle)*fs];
gyro_reading = gyro_rate + gyro_bias + gyro_noise;

% Simulate accelerometer
accel_noise = 2 * randn(size(t));
accel_reading = true_angle + accel_noise;

% Kalman Filter Initialization
x = [accel_reading(1); 0];
P = eye(2);
Q = [0.01 0; 0 0.003];
R = 4;
A = [1 -1/fs; 0 1];
B = [1/fs; 0];
H = [1 0];
fused_kalman = zeros(size(t));
fused_kalman(1) = x(1);

% Run Kalman Filter
for i = 2:length(t)
    x = A * x + B * gyro_reading(i);
    P = A * P * A' + Q;
    K = P * H' / (H * P * H' + R);
    x = x + K * (accel_reading(i) - H * x);
    P = (eye(2) - K * H) * P;
    fused_kalman(i) = x(1);
end

%% GUI Setup
figure('Name', 'Sensor Fusion GUI', 'NumberTitle', 'off', 'Position', [100 100 1000 500]);

% Subplot 1: Signals
subplot(1,2,1);
h_true = plot(t, true_angle, 'k', 'LineWidth', 1.5); hold on;
h_kalman = plot(t, fused_kalman, 'm', 'LineWidth', 1.5);
h_error = plot(t, fused_kalman - true_angle, 'r:');
legend('True Angle', 'Kalman Estimate', 'Error');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Angle vs Time');
grid on;

% Subplot 2: Pointer
subplot(1,2,2);
theta_disk = linspace(0, 2*pi, 100);
plot(cos(theta_disk), sin(theta_disk), 'k:'); hold on;
h_pointer = plot([0 cosd(fused_kalman(1))], [0 sind(fused_kalman(1))], 'r', 'LineWidth', 3);
axis equal;
axis([-1.2 1.2 -1.2 1.2]);
title_text = title(sprintf('Estimated Angle: %.2f°', fused_kalman(1)));
grid on;

% Animate pointer and update title
for k = 1:5:length(t)
    angle_deg = fused_kalman(k);
    set(h_pointer, 'XData', [0 cosd(angle_deg)], 'YData', [0 sind(angle_deg)]);
    set(title_text, 'String', sprintf('Estimated Angle: %.2f° | Error: %.2f°', angle_deg, angle_deg - true_angle(k)));
    drawnow;
end
