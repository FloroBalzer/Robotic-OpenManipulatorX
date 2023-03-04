%Base actuator ID11
%ID12-ID13 arm

% DH parameters
t0 = acos((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));
t1 = pi/2;
t2 = 0;
t3 = 0;
t4 = 0;
t5 = 0;

a0 = 0;       alpha0 = 0;     d1 = 0;         theta1 = t1;              % Link 1
a1 = 0;       alpha1 = 0;     d2 = 0.077;     theta2 = 0;               % Link 2
a2 = 0;       alpha2 = pi/2;  d3 = 0;         theta3 = t3-t0;           % Link 3
a3 = 0.130;   alpha3 = 0;     d4 = 0;         theta4 = t4+t0;           % Link 4
a4 = 0.124;   alpha4 = 0;     d5 = 0;         theta5 = t5;              % Link 4

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
%-------------------- Base to gripper -------------------
T02 = T01*T12;
T03 = T01*(T12*T23);
T04 = T01*(T12*(T23*T34));
T05 = T01*(T12*(T23*(T34*T45)));
T35 = T34*T45;


X1_x = linspace(T02(1,4),T02(1,4)+0.1,100);
X1_y = 0;
X1_z = 0;

Y1_x = 0;
Y1_y = linspace(T02(2,4),T02(2,4)+0.1,100);
Y1_z = 0;

Z1_x = 0;
Z1_y = 0;
Z1_z = linspace(T02(3,4),T02(3,4)+0.1,100);

%Plotting joint axes
grid on

% % Joint 1
line(X1_x,X1_y,X1_z, 'LineWidth', 0.5, 'Color', 'r') %Joint 1 x-axis
line(Y1_x,Y1_y,Y1_z, 'LineWidth', 0.5, 'Color', 'b') %Joint 1 y-axis
line(Z1_x,Z1_y,Z1_z, 'LineWidth', 0.5, 'Color', 'g') %Joint 1 z-axis

xlim([-0.075,0.3]); % [-3,12]
ylim([-0.2,0.2]); % [-8,8]
zlim([0, 0.5]);
view(3);
