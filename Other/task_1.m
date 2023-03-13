%Base actuator ID11
%ID12-ID13 arm

% DH parameters
t0 = acosd((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));
t1 = pi/2;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;

a0 = 0;       alpha0 = 0;     d1 = 0;         theta1 = t1;              % Joint 1
a1 = 0;       alpha1 = 0;     d2 = 0.077;     theta2 = 0;               % Joint 2
a2 = 0;       alpha2 = pi/2;  d3 = 0;         theta3 = t2-t0;           % Joint 3
a3 = 0.130;   alpha3 = 0;     d4 = 0;         theta4 = t3+t0;           % Joint 4
a4 = 0.124;   alpha4 = 0;     d5 = 0;         theta5 = t4;              % Joint 5
a5 = 0.126;   alpha5 = 0;     d6 = 0;         theta6 = t5;              % Joint 6

% Transformation matrix
%------------------------ Link 1 ------------------------
T01 = [cos(theta1),                 -sin(theta1),               0,              a0;
       sin(theta1)*cos(alpha0),     cos(theta1)*cos(alpha0),    -sin(alpha0),   -sin(alpha0)*d1;
       sin(theta1)*sin(alpha0),     cos(theta1)*sin(alpha0),    cos(alpha0),    cos(alpha0)*d1;
       0,                           0,                          0,              1];
%------------------------ Link 2 ------------------------
T12 = [cos(theta2),                 -sin(theta2),               0,              a1;
       sin(theta2)*cos(alpha1),     cos(theta2)*cos(alpha1),    -sin(alpha1),   -sin(alpha1)*d2;
       sin(theta2)*sin(alpha1),     cos(theta2)*sin(alpha1),    cos(alpha1),    cos(alpha1)*d2;
       0,                           0,                          0,              1];
%------------------------ Link 3 ------------------------
T23 = [cos(theta3),                 -sin(theta3),               0,              a2;
       sin(theta3)*cos(alpha2),     cos(theta3)*cos(alpha2),    -sin(alpha2),   -sin(alpha2)*d3;
       sin(theta3)*sin(alpha2),     cos(theta3)*sin(alpha2),    cos(alpha2),    cos(alpha2)*d3;
       0,                           0,                          0,              1];
%------------------------ Link 4 ------------------------
T34 = [cos(theta4),                 -sin(theta4),               0,              a3;
       sin(theta4)*cos(alpha3),     cos(theta4)*cos(alpha3),    -sin(alpha3),   -sin(alpha3)*d4;
       sin(theta4)*sin(alpha3),     cos(theta4)*sin(alpha3),    cos(alpha3),    cos(alpha3)*d4;
       0,                           0,                          0,              1];
%------------------------ Link 5 ------------------------
T45 = [cos(theta5),                 -sin(theta5),               0,              a4;
       sin(theta5)*cos(alpha4),     cos(theta5)*cos(alpha4),    -sin(alpha4),   -sin(alpha4)*d5;
       sin(theta5)*sin(alpha4),     cos(theta5)*sin(alpha4),    cos(alpha4),    cos(alpha4)*d5;
       0,                           0,                          0,              1];
%------------------------ Link 6 ------------------------
T45 = [cos(theta5),                 -sin(theta5),               0,              a4;
       sin(theta5)*cos(alpha4),     cos(theta5)*cos(alpha4),    -sin(alpha4),   -sin(alpha4)*d5;
       sin(theta5)*sin(alpha4),     cos(theta5)*sin(alpha4),    cos(alpha4),    cos(alpha4)*d5;
       0,                           0,                          0,              1];
%----------- Transformation from base to joint ----------
T02 = T01*T12;
T03 = T01*(T12*T23);
T04 = T01*(T12*(T23*T34));
T05 = T01*(T12*(T23*(T34*T45)));



