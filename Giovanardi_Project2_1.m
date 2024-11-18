clear();
clc();
close all;

% Parameters
c1 = 1;
c2 = 9;
simulationStepTime = 1/25;
simulationTime = 10.0;
time = 0.0;

% Initial configuration [x; y; theta]
q = [0.0; 0.0; 0.0];

% State matrix and distance of B point
k = [20 0; 0 20];
b = 0.5; 

d = 0.4; % Distance between wheels
r = 0.2; % Wheels radius
w = [ 0 ; 0 ]; % Angular velocities wr and wl

% Vectors to store robot's trajectory
x_trajectory = [];
y_trajectory = [];

% Simulation
while time < simulationTime
    % Desired positions and velocities
    x_des = c1*cos(c2 * time);
    y_des = c1*sin(c2 * time);
    x_des_dot = -c2*(c1*sin(c2 * time));
    y_des_dot = c2*(c1*cos(c2 * time));

    % Current position
    x = q(1);
    y = q(2);
    theta = q(3);

    x_trajectory = [x_trajectory, q(1)];
    y_trajectory = [y_trajectory, q(2)];
    
    % B virtual point (xB, yB)
    xB = x + b * cos(theta);
    yB = y + b * sin(theta);

    % Calculate vdx and vdy
    vdx = x_des_dot + k(1,1) * (x_des - xB);
    vdy = y_des_dot + k(2,2) * (y_des - yB);

    % Calculate control inputs [v; omega]
    T_inv = [cos(theta), sin(theta); -sin(theta)/b, cos(theta)/b];
    u = T_inv * [vdx; vdy];
    
    % Update model
    G = [cos(theta), 0.0; sin(theta), 0.0; 0.0, 1.0];
    q = q + G * u * simulationStepTime;

    % Advance simulation
    time = time + simulationStepTime;
    disp(strcat("time: ", num2str(time), " press ctrl + C to stop simulation"));

    % Show robot
    plotRobot(q);
    hold on;
    plot(x_des, y_des, 'go', 'MarkerFaceColor', 'g'); % Plot desired position
    hold off;
    drawnow;

   % Calculate angular velocities
   w = [1/r, -(d/r); -(1/r), -(d/r)]*u;


end

figure;
plot(x_trajectory, y_trajectory, '-b', 'LineWidth', 2);
hold on;
plot(x_trajectory(1), y_trajectory(1), 'go', 'MarkerFaceColor', 'g'); % Start point
plot(x_trajectory(end), y_trajectory(end), 'ro', 'MarkerFaceColor', 'r'); % End point
xlabel('X [m]');
ylabel('Y [m]');
title('Trajectory of the Robot');
grid on;
axis equal;

% Plot robot function
function plotRobot(q)
    w = 1.5;
    h = 1.2;

    T = [cos(q(3)), -sin(q(3)), 0.0, q(1); ...
         sin(q(3)), cos(q(3)), 0.0, q(2); ...
         0.0, 0.0, 1.0, 0.0; ...
         0.0, 0.0, 0.0, 1.0];

    footPrint1 = T * [- w / 2.0; - h / 2.0; 0.0; 1.0];
    footPrint2 = T * [w / 2.0; - h / 2.0; 0.0; 1.0];
    footPrint3 = T * [w / 2.0; h / 2.0; 0.0; 1.0];
    footPrint4 = T * [- w / 2.0; h / 2.0; 0.0; 1.0];

    head1 = T * [0.3; - 0.3; 0.0; 1.0];
    head2 = T * [w / 2.0; 0.0; 0.0; 1.0];
    head3 = T * [0.3; 0.3; 0.0; 1.0];

    leftWheel1 = T * [- w / 4.0; h / 1.8; 0.0; 1.0];
    leftWheel2 = T * [w / 4.0; h / 1.8; 0.0; 1.0];
    leftWheel3 = T * [w / 4.0; h / 1.8 + 0.2; 0.0; 1.0];
    leftWheel4 = T * [- w / 4.0; h / 1.8 + 0.2; 0.0; 1.0];

    rightWheel1 = T * [- w / 4.0; - h / 1.8; 0.0; 1.0];
    rightWheel2 = T * [w / 4.0; - h / 1.8; 0.0; 1.0];
    rightWheel3 = T * [w / 4.0; - h / 1.8 - 0.2; 0.0; 1.0];
    rightWheel4 = T * [- w / 4.0; - h / 1.8 - 0.2; 0.0; 1.0];

    footPrint = [footPrint1, footPrint2, footPrint3, footPrint4];
    head = [head1, head2, head3];
    leftWheel = [leftWheel1, leftWheel2, leftWheel3, leftWheel4];
    rightWheel = [rightWheel1, rightWheel2, rightWheel3, rightWheel4];

    plot(polyshape(footPrint(1,:), footPrint(2,:)),'EdgeColor','k','FaceColor',[0 0.4470 0.7410],'FaceAlpha',0.8);
    hold on
    plot(polyshape(head(1,:), head(2,:)),'EdgeColor','k','FaceColor',[0.9290 0.6940 0.1250],'FaceAlpha',0.8);
    plot(polyshape(leftWheel(1,:), leftWheel(2,:)),'EdgeColor','k','FaceColor','k','FaceAlpha',1.0);
    plot(polyshape(rightWheel(1,:), rightWheel(2,:)),'EdgeColor','k','FaceColor','k','FaceAlpha',1.0);
    plot(q(1), q(2),'Or','MarkerEdgeColor','k','MarkerFaceColor','r','MarkerSize', 8);
    hold off
    grid on
    axis equal
    xlabel("X [m]")
    ylabel("Y [m]")
    xlim([-8.0, 8.0]);
    ylim([-8.0, 8.0]);
    drawnow
end
