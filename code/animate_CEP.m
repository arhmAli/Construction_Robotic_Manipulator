L1 = Link('d', 0.1, 'a', 0, 'alpha', pi/2);
L2 = Link('d', 0, 'a', 0.5, 'alpha', 0);
L3 = Link('d', 0, 'a', 0.2, 'alpha', 0);
L4 = Link('d', 0.5, 'a', 0, 'alpha', -pi/2);
L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
L6 = Link('d', 0.1, 'a', 0, 'alpha', 0, 'offset', pi/2);
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '2019MC253');
q_min = [-pi/2 -pi/2 -pi/2 -pi/2 -pi/2 -pi/2];
q_max = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2];
q0 = zeros(1,6);
t = linspace(0, 10, 100);  % Time vector
x = sin(t);  % X-coordinates of desired trajectory
y = cos(t);  % Y-coordinates of desired trajectory
z = 0.5*t;  % Z-coordinates of desired trajectory
XYZ = [x' y' z'];
q_traj = jtraj(q0, q_max, length(t));
robot.plot(q_traj, 'trail', 'r-', 'fps', 30);
