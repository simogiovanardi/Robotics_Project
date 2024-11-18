clear;
clc;
close all;

c1 = 1;
c2 = 9;
c3 = 5;
c4 = 4;
c5 = 4;

% Robot
startConfiguration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
goalConfiguration = 1 / (2 * pi) .* [c1, c2, c3, c4, c5, c1];
robot = loadrobot("universalUR5");
robot.DataFormat = "row";

% Planning
rrt = manipulatorRRT(robot, {}, "IgnoreSelfCollision", true, MaxConnectionDistance = 1);
rng(0); % Seed for repeatability
path = plan(rrt, startConfiguration, goalConfiguration); % 3x6 matrix

% Plotting the path
clf
for i = 1:size(path,1)
    show(robot, path(i,:));
    hold on
end
hold off

% Time and trajectory parameters
tf = 2;
t = linspace(0, tf, size(path, 1)); % Original time points
tt = 0:0.02:tf; % Evaluation time points

% Interpolating positions with cubic splines with zero velocities at the ends
q = zeros(length(tt), size(path, 2));
dq = zeros(length(tt), size(path, 2));
ddq = zeros(length(tt), size(path, 2));

for j = 1:size(path, 2)
    % Spline with clamped conditions
    pp = spline(t, [0, path(:, j).', 0]); % [0, ..., 0] ensures zero velocity at the ends
    q(:, j) = ppval(pp, tt);
    
    % Derivatives for velocity and acceleration
    dpp = fnder(pp, 1);
    ddpp = fnder(pp, 2);
    
    dq(:, j) = ppval(dpp, tt);
    ddq(:, j) = ppval(ddpp, tt);
end

% Plotting joint positions
figure('Name', 'Joint Positions');
plot(tt, q, 'LineWidth', 3);
xlabel('Time (s)');
ylabel('Position (rad)');
title('Joint Positions');
grid on;

% Plotting joint velocities
figure('Name', 'Joint Velocities');
plot(tt, dq, 'LineWidth', 3);
xlabel('Time (s)');
ylabel('Velocity (rad/s)');
title('Joint Velocities');
grid on;

% Plotting joint accelerations
figure('Name', 'Joint Accelerations');
plot(tt, ddq, 'LineWidth', 3);
xlabel('Time (s)');
ylabel('Acceleration (rad/s^2)');
title('Joint Accelerations');
grid on;
