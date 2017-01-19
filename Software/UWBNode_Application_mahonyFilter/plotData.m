clc;
clear;
close all;

% load data
load('imuData_20161220_192924.mat');
time = data(1, :) + data(2, :);
time = time - time(1);
yg   = data( 3: 5, :);
ya   = data( 6: 8, :);
ym   = data( 9:11, :);
att  = data(12:14, :);
q    = data(15:18, :);

dataInfo = [ sprintf('lens = %d', dataLens), sprintf('t = %.2fs', time(end)), dataIndex ]

% {
% check time
dt = fix((time(2:end) - time(1:end-1)) * 1e3 + 1e-5) / 1e3;
% plot(1:dataLens-1, dt);
index = find(dt ~= mode(dt));
errTime = [index, time(index), time(index + 1)]
%}

% {
fig1 = figure(1);

subplot(4, 1, 1);
hold on
grid on
plot(time, yg);
legend('gyr_x', 'gyr_y', 'gyr_z');
xlabel('time (s)');
ylabel('Angular velocity (dps)');

subplot(4, 1, 2);
hold on
grid on
plot(time, ya);
legend('acc_x', 'acc_y', 'acc_z');
xlabel('time (s)');
ylabel('Acceleration (m/s^2)');

subplot(4, 1, 3);
hold on
grid on
plot(time, ym);
legend('mag_x', 'mag_y', 'mag_z');
xlabel('time (s)');
ylabel('Magnetic (gauss)');

subplot(4, 1, 4);
hold on
grid on
plot(time, att);
legend('pitch', 'roll', 'yaw');
xlabel('time (s)');
ylabel('theta (deg)');
%}
