function [T01, T12, T23, T34,T02, T03, T04, T05] = ForwardKinematics(t2, t3, t4, t5)

% DH parameters
t0 = acosd((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));
t1 = 90;

a0 = 0;       alpha0 = 0;     d1 = 0;         theta1 = t1;              % Link 1
a1 = 0;       alpha1 = 0;     d2 = 0.077;     theta2 = t2;              % Link 2
a2 = 0.130;   alpha2 = pi/2;  d3 = 0;         theta3 = t3-t0;           % Link 3
a3 = 0.124;   alpha3 = 0;     d4 = 0;         theta4 = t4+t0;           % Link 4
a4 = 0.126;   alpha4 = 0;     d5 = 0;         theta5 = t5;  

T01 = Trans_Matrix(a0, alpha0, d1, theta1);
T12 = Trans_Matrix(a1, alpha1, d2, theta2);
T23 = Trans_Matrix(a2, alpha2, d3, theta3);
T34 = Trans_Matrix(a3, alpha3, d4, theta4);
T45 = Trans_Matrix(a4, alpha4, d5, theta5);

T02 = T01*T12;
T03 = T01*(T12*T23);
T04 = T01*(T12*(T23*T34));
T05 = T01*(T12*(T23*(T34*T45)));
end