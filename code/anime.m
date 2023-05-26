% Define the robot model
L1 = 5;
L2 = 5;
L3 = 5;
L4 = 5;
L5 = 5;
L6 = 5;

% DH parameters
alpha = [0, pi/2, 0, 0, pi/2, 0];
a = [0, 0, L2, L3, 0, 0];
d = [L1, 0, 0, L4, L5, L6];
theta = [0, 0, 0, 0, 0, 0];

% Define the robot
robot = SerialLink([theta', d', a', alpha'],'name','2019 MC 253');

% Define the trajectory
t = linspace(0, 5, 100);
q1 = 0.5*sin(2*pi*t);
q2 = 0.5*cos(2*pi*t);
q3 = 0.5*sin(4*pi*t);
q4 = 0.5*cos(4*pi*t);
q5 = 0.5*sin(6*pi*t);
q6 = 0.5*cos(6*pi*t);
q = [q1', q2', q3', q4', q5', q6'];

% Compute the maximum acceleration
qd = diff(q)./diff(t)';
qdd = diff(qd)./diff(t(1:end-1))';
amax = max(sqrt(sum(qdd.^2, 2)));

% Plot the trajectory
figure;
robot.plot(q,'trail','r');
title('Trajectory of the robot');
%name('2019-MC-253');

% Display the maximum acceleration
fprintf('Maximum acceleration: %f\n', amax);




% Define robot and trajectory
L1 = 1; L2 = 1; L3 = 1; L4 = 1; L5 = 1; L6 = 1;
d1 = L1; d2 = 0; d3 = 0; d4 = L4; d5 = L5; d6 = L6;
alpha = [0, pi/2, 0, 0, pi/2, 0];
a = [0, 0, L2, L3, 0, 0];
d = [d1, d2, d3, d4, d5, d6];
theta = [0, 0, 0, 0, 0, 0];
robot = SerialLink([theta' d' a' alpha']);

% Define trajectory and plot in red color
q0 = [0 0 0 0 0 0];
q1 = [0.5 -0.5 0.5 0.5 -0.5 0.5];
q2 = [1 1 1 1 1 1];
q3 = [-1 -1 -1 -1 -1 -1];
traj = jtraj(q0, q1, q2, q3, 50);
plot(robot, traj);

% Compute maximum acceleration
t = 0:0.02:1;
[q, qd, qdd] = jtraj(q0, q1, t);
amax = max(sqrt(sum(qdd.^2, 2)));
% Check if the robot is in a singular configuration
is_singular = find_singularity(robot, q);

% Display the result
if is_singular
    disp('The robot is in a singular configuration.');
else
    disp('The robot is not in a singular configuration.');
end
% Display maximum acceleration
fprintf('Maximum acceleration: %.2f\n', amax);
clear all
close all

L1 = 1; % meters
L2 = 1; % meters
L3 = 1; % meters
L4 = 1; % meters
L5 = 1; % meters
L6 = 1; % meters

robot = SerialLink([
    Revolute('d', L1, 'a', 0, 'alpha', 0)
    Revolute('d', 0, 'a', 0, 'alpha', pi/2)
    Revolute('d', 0, 'a', L2, 'alpha', 0)
    Revolute('d', L3, 'a', L4, 'alpha', 0)
    Revolute('d', 0, 'a', 0, 'alpha', pi/2)
    Revolute('d', L5, 'a', L6, 'alpha', 0)], 'name', 'my_robot');

q_min = [-pi/2, -pi/2, -pi/2, -pi/2, -pi/2, -pi/2];
q_max = [ pi/2,  pi/2,  pi/2,  pi/2,  pi/2,  pi/2];
steps = 20;

[q1, q2, q3, q4, q5, q6] = ndgrid(linspace(q_min(1), q_max(1), steps), ...
    linspace(q_min(2), q_max(2), steps), ...
    linspace(q_min(3), q_max(3), steps), ...
    linspace(q_min(4), q_max(4), steps), ...
    linspace(q_min(5), q_max(5), steps), ...
    linspace(q_min(6), q_max(6), steps));

q = [q1(:), q2(:), q3(:), q4(:), q5(:), q6(:)];
num_points = size(q, 1);

% Compute the workspace
workspace = zeros(num_points, 3);
for i = 1:num_points
    T = robot.fkine(q(i, :));
    workspace(i, :) = T(1:3, 4)';
end

% Plot the workspace
figure();
plot3(workspace(:, 1), workspace(:, 2), workspace(:, 3), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace of 6-DOF Robot');
grid on;

