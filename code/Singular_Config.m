% Define the DH parameters of the robot
L1 = 1; L2 = 1; L3 = 1; L4 = 1; L5 = 1; L6 = 1;
alpha = [0 pi/2 0 0 pi/2 0];
a = [0 0 L2 L3 0 0];
d = [L1 0 0 L4 L5 L6];
theta = [0 0 0 0 0 0];
robot = SerialLink([theta' d' a' alpha'], 'name', '2019-MC-253');
q = rand(1, 6) .* [2*pi 2*pi 2*pi 2*pi 2*pi 2*pi] - [pi pi pi pi pi pi];
J = robot.jacob0(q);
s = svd(J);
threshold = 1e-6;
if any(s < threshold)
    disp('The robot is in a singular position');
else
    disp('The robot is not in a singular position');
end
q1 = linspace(-pi/2, pi/2, 50);
q2 = linspace(-pi/2, pi/2, 50);
q3 = linspace(0, pi/2, 50);
q4 = linspace(-pi/2, pi/2, 50);
q5 = linspace(-pi/2, pi/2, 50);
q6 = linspace(-pi/2, pi/2, 50);
% Define the robot model
L1 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/2 pi/2]);
L2 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi/2 pi/2]);
L3 = Link('d', 0.5, 'a', 0, 'alpha', -pi/2, 'qlim', [0 pi/2]);
L4 = Link('d', 0.5, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/2 0]);
L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi/2 pi/2]);
L6 = Link('d', 0.1, 'a', 0, 'alpha', 0, 'qlim', [-pi/2 pi/2]);
robot.teach()
robot.plot([0 0 0 0 0 0], 'workspace', [-3 3 -3 3 -3 3]);
