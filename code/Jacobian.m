syms q1 q2 q3 q4 q5 q6 L1 L2 L3 L4 L5 L6
L1 = 5; L2 = 5; L3 = 5; L4 = 5; L5 = 5; L6 = 5;
%theta = [0, 0, 0, 0, 0, 0];

alpha = [0, pi/2, 0, 0, pi/2, 0];
a = [0, 0, L2, L3, 0, 0];
d = [L1, 0, 0, L4, L5, L6];

theta = [q1, q2, q3, q4, q5, q6];

T01 = [cos(theta(1)), -sin(theta(1)), 0, a(1);
       sin(theta(1))*cos(alpha(1)), cos(theta(1))*cos(alpha(1)), -sin(alpha(1)), -d(1)*sin(alpha(1));
       sin(theta(1))*sin(alpha(1)), cos(theta(1))*sin(alpha(1)), cos(alpha(1)), d(1)*cos(alpha(1));
       0, 0, 0, 1];

T12 = [cos(theta(2)), -sin(theta(2)), 0, a(2);
       sin(theta(2))*cos(alpha(2)), cos(theta(2))*cos(alpha(2)), -sin(alpha(2)), -d(2)*sin(alpha(2));
       sin(theta(2))*sin(alpha(2)), cos(theta(2))*sin(alpha(2)), cos(alpha(2)), d(2)*cos(alpha(2));
       0, 0, 0, 1];

T23 = [cos(theta(3)), -sin(theta(3)), 0, a(3);
       sin(theta(3))*cos(alpha(3)), cos(theta(3))*cos(alpha(3)), -sin(alpha(3)), -d(3)*sin(alpha(3));
       sin(theta(3))*sin(alpha(3)), cos(theta(3))*sin(alpha(3)), cos(alpha(3)), d(3)*cos(alpha(3));
       0, 0, 0, 1];

T34 = [cos(theta(4)), -sin(theta(4)), 0, a(4);
       sin(theta(4))*cos(alpha(4)), cos(theta(4))*cos(alpha(4)), -sin(alpha(4)), -d(4)*sin(alpha(4));
       sin(theta(4))*sin(alpha(4)), cos(theta(4))*sin(alpha(4)), cos(alpha(4)), d(4)*cos(alpha(4));
       0, 0, 0, 1];

T45 = [cos(theta(5)), -sin(theta(5)), 0, a(5);
       sin(theta(5))*cos(alpha(5)), cos(theta(5))*cos(alpha(5)), -sin(alpha(5)), -d(5)*sin(alpha(5));
       sin(theta(5))*sin(alpha(5)), cos(theta(5))*sin(alpha(5)), cos(alpha(5)), d(5)*cos(alpha(5));
       0, 0, 0, 1];
T56 = [cos(theta(6)), -sin(theta(6)), 0, a(6);
       sin(theta(6))*cos(alpha(6)), cos(theta(6))*cos(alpha(6)), -sin(alpha(6)), -d(6)*sin(alpha(6));
       sin(theta(6))*sin(alpha(6)), cos(theta(6))*sin(alpha(6)), cos(alpha(6)), d(6)*cos(alpha(6));
       0, 0, 0, 1];

% Forward Kinematics for the complete chain
T06 = T01 * T12 * T23 * T34 * T45 * T56;

% Velocity Kinematics
J = simplify([diff(T06(1,4),theta(1)) diff(T06(1,4),theta(2)) diff(T06(1,4),theta(3)) diff(T06(1,4),theta(4)) diff(T06(1,4),theta(5)) diff(T06(1,4),theta(6));
               diff(T06(2,4),theta(1)) diff(T06(2,4),theta(2)) diff(T06(2,4),theta(3)) diff(T06(2,4),theta(4)) diff(T06(2,4),theta(5)) diff(T06(2,4),theta(6));
               diff(T06(3,4),theta(1)) diff(T06(3,4),theta(2)) diff(T06(3,4),theta(3)) diff(T06(3,4),theta(4)) diff(T06(3,4),theta(5)) diff(T06(3,4),theta(6))]);


% Calculate the singular values of the Jacobian
sigma = svd(J);

% Find the smallest singular value
%min_sigma = min(sigma);

% Check if the Jacobian is singular
if min_sigma < eps
    disp('The Jacobian is singular');
else
    disp('The Jacobian is not singular');
end
