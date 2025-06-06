
% Sensor Fusion GUI with Start/Stop, Manual Slider, and Real-Time Noise Toggle
function sensor_fusion_gui_realtime_noise
    %% Parameters
    fs = 100;
    T = 10;
    t = 0:1/fs:T;

    % Ground truth angle
    true_angle = 45 * sin(2 * pi * 0.2 * t);

    % Initial sensor signals (with noise)
    [gyro_reading, accel_reading] = generate_noisy_signals(true_angle, fs, false);

    % Kalman filter setup
    fused_kalman = run_kalman_filter(gyro_reading, accel_reading, fs);

    %% GUI Setup
    f = figure('Name', 'Sensor Fusion GUI – Real-Time Noise Toggle', 'Position', [100 100 1100 550]);

    % Plot axes
    ax1 = subplot(1,2,1);
    h_true = plot(t, true_angle, 'k', 'LineWidth', 1.5); hold on;
    h_kalman = plot(t, fused_kalman, 'm', 'LineWidth', 1.5);
    h_error = plot(t, fused_kalman - true_angle, 'r:');
    legend('True Angle', 'Kalman Estimate', 'Error');
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Angle vs Time');
    grid on;

    ax2 = subplot(1,2,2);
    theta_disk = linspace(0, 2*pi, 100);
    plot(cos(theta_disk), sin(theta_disk), 'k:'); hold on;
    h_pointer = plot([0 cosd(fused_kalman(1))], [0 sind(fused_kalman(1))], 'r', 'LineWidth', 3);
    axis equal;
    axis([-1.2 1.2 -1.2 1.2]);
    title_text = title(sprintf('Estimated Angle: %.2f°', fused_kalman(1)));
    grid on;

    % Controls
    start_btn = uicontrol('Style', 'pushbutton', 'String', 'Start', ...
        'Position', [920 480 80 30], 'Callback', @start_callback);
    stop_btn = uicontrol('Style', 'pushbutton', 'String', 'Stop', ...
        'Position', [1000 480 80 30], 'Callback', @stop_callback);

    uicontrol('Style', 'text', 'Position', [920 440 160 20], 'String', 'Manual Position');
    manual_slider = uicontrol('Style', 'slider', ...
        'Min', 1, 'Max', length(t), 'Value', 1, ...
        'Position', [920 420 160 20], 'Callback', @slider_callback);

    noise_checkbox = uicontrol('Style', 'checkbox', ...
        'String', 'Add Real-Time Noise', 'Value', 0, ...
        'Position', [920 390 160 20]);

    running = false;
    k = 1;

    function start_callback(~, ~)
        running = true;
        while running && k <= length(t)
            if get(noise_checkbox, 'Value')
                [gyro_reading, accel_reading] = generate_noisy_signals(true_angle, fs, true);
                fused_kalman = run_kalman_filter(gyro_reading, accel_reading, fs);
                set(h_kalman, 'YData', fused_kalman);
                set(h_error, 'YData', fused_kalman - true_angle);
            end
            update_pointer(k);
            pause(0.02);
            k = k + 5;
        end
    end

    function stop_callback(~, ~)
        running = false;
    end

    function slider_callback(src, ~)
        k = round(get(src, 'Value'));
        update_pointer(k);
    end

    function update_pointer(index)
        angle_deg = fused_kalman(index);
        set(h_pointer, 'XData', [0 cosd(angle_deg)], 'YData', [0 sind(angle_deg)]);
        set(title_text, 'String', sprintf('Estimated Angle: %.2f° | Error: %.2f°', angle_deg, angle_deg - true_angle(index)));
    end

    function [gyro, accel] = generate_noisy_signals(angle, fs, noisy)
        gyro_bias = 0.2;
        gyro_noise = 0.5 * randn(size(angle)) * noisy;
        gyro_rate = [0, diff(angle)*fs];
        gyro = gyro_rate + gyro_bias + gyro_noise;

        accel_noise = 2 * randn(size(angle)) * noisy;
        accel = angle + accel_noise;
    end

    function fused = run_kalman_filter(gyro_reading, accel_reading, fs)
        x = [accel_reading(1); 0];
        P = eye(2);
        Q = [0.01 0; 0 0.003];
        R = 4;
        A = [1 -1/fs; 0 1];
        B = [1/fs; 0];
        H = [1 0];

        fused = zeros(size(accel_reading));
        fused(1) = x(1);
        for i = 2:length(accel_reading)
            x = A * x + B * gyro_reading(i);
            P = A * P * A' + Q;
            K = P * H' / (H * P * H' + R);
            x = x + K * (accel_reading(i) - H * x);
            P = (eye(2) - K * H) * P;
            fused(i) = x(1);
        end
    end
video = VideoWriter('sensor_fusion_demo.mp4', 'MPEG-4');
video.FrameRate = 30;
open(video);

while running && k <= length(t)
    if get(noise_checkbox, 'Value')
        [gyro_reading, accel_reading] = generate_noisy_signals(true_angle, fs, true);
        fused_kalman = run_kalman_filter(gyro_reading, accel_reading, fs);
        set(h_kalman, 'YData', fused_kalman);
        set(h_error, 'YData', fused_kalman - true_angle);
    end
    update_pointer(k);
    frame = getframe(gcf);   % Capture current GUI frame
    writeVideo(video, frame); % Write to file
    pause(0.02);
    k = k + 5;
end

close(video);  % Finalize the video

end
