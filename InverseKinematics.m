function [t1, t2, t3, t4] = InverseKinematics(T05)

a0 = 0;       alpha0 = 0;     d1 = 0.077;             % Link 1
a1 = 0;       alpha1 = 90;    d2 = 0;                 % Link 2
a2 = 0.128;   alpha2 = 0;     d3 = 0;                 % Link 3
a3 = 0.148;   alpha3 = 0;     d4 = 0;                 % Link 4
a4 = 0.126;   alpha4 = 0;     d5 = 0;                 % Gripper
    
tool_s = sqrt(T05(1,4).^2 + T05(2,4).^2);
tool_t = T05(3,4)-d1;

final_x_angle = -asind(T05(3,1));
joint4_s = tool_s - a4 *cosd(final_x_angle);
joint4_t = tool_t - a4 * sind(final_x_angle);
fprintf("joint4_s: %4.2f ", joint4_s);
fprintf("joint4_t: %4.2f\n", joint4_t);

%theta3 = acosd((joint4_s.^2 + joint4_t.^2 - a2.^2 - a3.^2)/(2*a2*a3));
theta3 = -acosd((joint4_s.^2 + joint4_t.^2 - a2.^2 - a3.^2)/(2*a2*a3));

theta2_1 = acosd(((a2+a3*cosd(theta3))*joint4_s+(a3*sind(theta3))*joint4_t)/(joint4_s.^2 + joint4_t.^2));
theta2_2 = asind(((a2+a3*cosd(theta3))*joint4_t+(a3*sind(theta3))*joint4_s)/(joint4_s.^2 + joint4_t.^2));
theta2 = theta2_1;
fprintf("theta2_1: %4.2f ", theta2_1);
fprintf("theta2_2: %4.2f\n", theta2_2);

theta4 = final_x_angle - (theta2+theta3);

if (T05(1,4)>0 && T05(2,4)>0)
    theta1 = atand(T05(2,4)/T05(1,4));
elseif (T05(1,4)<0 && T05(2,4)>0)
    theta1 = atand(T05(2,4)/T05(1,4))+180;
elseif (T05(1,4)<0 && T05(2,4)<0)
    theta1 = atand(T05(2,4)/T05(1,4))-180;
else
    theta1 = atand(T05(2,4)/T05(1,4));
    


t1 = theta1+180;
t2 = theta2 + 79 + atand(0.024/0.128);
t3 = theta3 +101 - atand(0.024/0.128);
t4 = theta4;

fprintf("t1: %4.2f\n", t1);
fprintf("t2: %4.2f\n", t2);
fprintf("t3: %4.2f\n", t3);
fprintf("t4: %4.2f\n", t4);

% fprintf("I_Theta1: %4.2f ", theta1);
% fprintf("I_Theta2: %4.2f ", theta2);
% fprintf("I_Theta3: %4.2f ", theta3);
% fprintf("I_Theta4: %4.2f\n", theta4);

    
end
