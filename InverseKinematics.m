 function [t1, t2, t3, t4] = InverseKinematics(T05)

% DH parameters
t0 = acosd((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));
a0 = 0;         alpha0 = 0;     d1 = 0.077;  % Link 1
a1 = 0;         alpha1 = 90;    d2 = 0;      % Link 2
a2 = 0.128;     alpha2 = 0;     d3 = 0;      % Link 3
a3 = 0.148;     alpha3 = 0;     d4 = 0;      % Link 4
a4 = 0.126;     alpha4 = 0;     d5 = 0;      % Gripper

% Extract the position and orientation from T05
position = T05(1:3, 4);
orientation = T05(1:3, 1:3);

% Compute the wrist center position (WC) and the orientation of link 3 (R)
WC = position - (a4 * orientation(:,3));
R = orientation(:, 1:2);

% Compute the joint angles t1, t2, t3
t1 = atan2d(WC(2), WC(1));
t3 = acosd((norm(WC)^2 - (d1^2 + d4^2 + a2^2 + a3^2)) / (2 * a2 * a3));
t2 = atan2d(WC(3) - d1, sqrt(WC(1)^2 + WC(2)^2)) - atan2d(a3 * sind(t3), a2 + a3*cosd(t3));

% Compute the orientation of link 1 and link 2
R0_3 = Trans_Matrix(a0, alpha0, d1, t1) * Trans_Matrix(a1, alpha1, d2, t2) * Trans_Matrix(a2, alpha2, d3, t3);
R3_6 = R0_3' * orientation;
t4 = atan2d(R3_6(2, 3), R3_6(1, 3));

end
