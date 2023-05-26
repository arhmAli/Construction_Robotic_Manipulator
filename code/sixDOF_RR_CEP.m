%6DOF Articulated Manipulator with spherical wrist.

% Define the robot geometry
L1 = 5; % length of first link
L2 = 5; % length of second link
L3 = 5; % length of third link
L4 = 5; % length of fourth link
L5 = 5; % length of fifth link
L6 = 5; % length of sixth link

L(1) = Link([0 L1 0 pi/2]);
L(2) = Link([0 0 L2 0]);
L(3) = Link([0 0 L3 0]);
L(4) = Link([0 L4 0 pi/2]);
L(5) = Link([0 L5 0 -pi/2]);
L(6) = Link([0 0 L6 0]);

% Define the joint limits
L(1).qlim = [-pi pi];
L(2).qlim = [-pi pi];
L(3).qlim = [-pi pi];
L(4).qlim = [-pi pi];
L(5).qlim = [-pi pi];
L(6).qlim = [-pi pi];

% Define the home position
q0 = [0 0 0 0 0 0];

% Define the tool frame
tool = transl([0 0 L6]);

% Define the base frame
base = transl([0 0 L1]);

% Create the robot object
robot = SerialLink(L, 'name', '2019MC253', 'tool', tool, 'base', base);

% Test the robot
robot.plot(q0);
robot.animate(q0);
\











% Define link lengths
L1 = 5; L2 = 5; L3 = 5; L4 = 5; L5 = 5; L6 = 5;

% Define DH parameters
alpha = [0, pi/2, 0, 0, pi/2, 0];
a = [0, 0, L2, L3, 0, 0];
d = [L1, 0, 0, L4, L5, L6];
theta = [0, 0, 0, 0, 0, 0];

% Calculate DH transformation matrices
A1 = [cos(theta(1)), -sin(theta(1))*cos(alpha(1)), sin(theta(1))*sin(alpha(1)), a(1)*cos(theta(1));
      sin(theta(1)), cos(theta(1))*cos(alpha(1)), -cos(theta(1))*sin(alpha(1)), a(1)*sin(theta(1));
      0, sin(alpha(1)), cos(alpha(1)), d(1);
      0, 0, 0, 1];

A2 = [cos(theta(2)), -sin(theta(2))*cos(alpha(2)), sin(theta(2))*sin(alpha(2)), a(2)*cos(theta(2));
      sin(theta(2)), cos(theta(2))*cos(alpha(2)), -cos(theta(2))*sin(alpha(2)), a(2)*sin(theta(2));
      0, sin(alpha(2)), cos(alpha(2)), d(2);
      0, 0, 0, 1];

A3 = [cos(theta(3)), -sin(theta(3))*cos(alpha(3)), sin(theta(3))*sin(alpha(3)), a(3)*cos(theta(3));
      sin(theta(3)), cos(theta(3))*cos(alpha(3)), -cos(theta(3))*sin(alpha(3)), a(3)*sin(theta(3));
      0, sin(alpha(3)), cos(alpha(3)), d(3);
      0, 0, 0, 1];

A4 = [cos(theta(4)), -sin(theta(4))*cos(alpha(4)), sin(theta(4))*sin(alpha(4)), a(4)*cos(theta(4));
      sin(theta(4)), cos(theta(4))*cos(alpha(4)), -cos(theta(4))*sin(alpha(4)), a(4)*sin(theta(4));
      0, sin(alpha(4)), cos(alpha(4)), d(4);
      0, 0, 0, 1];

A5 = [cos(theta(5)), -sin(theta(5))*cos(alpha(5)), sin(theta(5))*sin(alpha(5)), a(5)*cos(theta(5));
      sin(theta(5)), cos(theta(5))*cos(alpha(5)), -cos(theta(5))*sin(alpha(5)), a(5)*sin(theta(5));
      0, sin(alpha(5)), cos(alpha(5)), d(5);
      0, 0, 0, 1];
A6 = [cos(theta(6)), -sin(theta(6))*cos(alpha(6)), sin(theta(6))*sin(alpha(6)), a(6)*cos(theta(6));
sin(theta(6)), cos(theta(6))*cos(alpha(6)), -cos(theta(6))*sin(alpha(6)), a(6)*sin(theta(6));
0, sin(alpha(6)), cos(alpha(6)), d(6);
0, 0, 0, 1];
T06 = A1*A2*A3*A4*A5*A6;


