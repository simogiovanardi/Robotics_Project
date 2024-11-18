clear();
clc();
close all;

c1 = 1;
c2 = 9;
c3 = 5;
c4 = 4;
c5 = 4;

% Robot
startConfiguration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
goalConfiguration2 = 1 / (2 * pi) .* [c1, c1, c3, 0.0, 0.0, 0.0];
robot = loadrobot("universalUR5");
robot.DataFormat = "row";

%Obstacle 1
s1 = [0; 0; 0.4];
r1 = 0.2;
obstacle1 = collisionSphere(r1);
obstacle1Pose = eye(4);
obstacle1Pose(1:3,4) = s1;
obstacle1.Pose = obstacle1Pose;

%Obstacle 2
s2 = [0; 0; -0.4];
r2 = 0.1;
obstacle2 = collisionSphere(r2);
obstacle2Pose = eye(4);
obstacle2Pose(1:3,4) = s2;
obstacle2.Pose = obstacle2Pose;

%Environment
environment = {obstacle1, obstacle2};


rrt2 = manipulatorRRT(robot, environment, "IgnoreSelfCollision", true, MaxConnectionDistance = 0.5);
rng(0); % Seed for repeatability
path2 = plan(rrt2, startConfiguration, goalConfiguration2);

figure
for i = 1:size(path2,1)
    show(robot, path2(i,:));
    hold on
end
for i=1:length(environment)
    show(environment{i});
end
hold off

% Time and trajectory parameters
tf = 3;
t = linspace(0, tf, size(path2, 1)); % Original time points
tt = 0:0.02:tf; % Evaluation time points

q = zeros(length(tt), size(path2, 2));
dq = zeros(length(tt), size(path2, 2));
ddq = zeros(length(tt), size(path2, 2));

for j = 1:size(path2, 2)
    % Spline with clamped conditions
    pp = spline(t, [0, path2(:, j).', 0]); % [0, ..., 0] ensures zero velocity at the ends
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
