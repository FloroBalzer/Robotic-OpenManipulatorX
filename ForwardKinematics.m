function [T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(t1, t2, t3, t4)

% DH parameters
t0 = atand(0.024/0.128);

a0 = 0;       alpha0 = 0;     d1 = 0.077;     theta1 = t1;              % Link 1
a1 = 0;       alpha1 = 90;    d2 = 0;         theta2 = t2-t0-79;        % Link 2
a2 = 0.128;   alpha2 = 0;     d3 = 0;         theta3 = t3+t0-101;       % Link 3
a3 = 0.148;   alpha3 = 0;     d4 = 0;         theta4 = t4-180;          % Link 4
a4 = 0.126;   alpha4 = 0;     d5 = 0;         theta5 = 0;               % Gripper



%Tansformation Matrices
T01 = Trans_Matrix(a0, alpha0, d1, theta1);
T12 = Trans_Matrix(a1, alpha1, d2, theta2);
T23 = Trans_Matrix(a2, alpha2, d3, theta3);
T34 = Trans_Matrix(a3, alpha3, d4, theta4);
T45 = Trans_Matrix(a4, alpha4, d5, theta5);

T02 = T01*T12;
T03 = T01*(T12*T23);
T04 = T01*(T12*(T23*T34));
T05 = T01*(T12*(T23*(T34*T45)));
% fprintf("F_Theta1: %4.2f ", theta1);
% fprintf("F_Theta2: %4.2f ", theta2);
% fprintf("F_Theta3: %4.2f ", theta3);
% fprintf("F_Theta4: %4.2f\n ", theta4);

end