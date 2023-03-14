function [t1, t2, t3, t4] = InverseKinematics2(T05)
%% setup
t0 = acosd((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));

a0 = 0;      alpha0 = 0;     d1 = 7.7;               % Link 1
a1 = 0;      alpha1 = 90;    d2 = 0;                 % Link 2
a2 = 13.0;   alpha2 = 0;     d3 = 0;                 % Link 3
a3 = 13.6;   alpha3 = 0;     d4 = 0;                 % Link 4
a4 = 12.6;   alpha4 = 0;     d5 = 0;                 % Gripper

%switching endposition to st space
tool_s = sqrt(T05(1,4).^2 + T05(2,4).^2);
tool_t = T05(3,4)-d1;

%% helper formulae
final_x_angle = asind(T05(3,1));
joint4_s = tool_s - a4 *cosd(final_x_angle);
joint4_t = tool_t - a4 * sind(final_x_angle);

%% T3
theta3 = acosd(((joint4_s.^2 + (joint4_t).^2-a2.^2 - a3.^2))/(2*a2*a3));
t3 = (real(theta3 + 90 + t0));

%% T2
c2 = (a2+a3*cosd(theta3)*joint4_s + a3*sind(theta3)*joint4_t)/(joint4_s.^2 + joint4_t.^2);
s2 =  (a2+a3*cosd(theta3)*joint4_t + a3*sind(theta3)*joint4_s)/(joint4_s.^2 + joint4_t.^2);
theta2 = atand(s2/c2);
t2 = theta2 +90 - t0;

 %% T4
 theta4 = final_x_angle - (theta2+theta3);
 t4 = theta4;


%% T1
if (T05(1,4)>0 && T05(2,4)>0)
    theta1 = atand(T05(2,4)/T05(1,4));
elseif (T05(1,4)<0 && T05(2,4)>0)
    theta1 = atand(T05(2,4)/T05(1,4))+180;
elseif (T05(1,4)<0 && T05(2,4)<0)
    theta1 = atand(T05(2,4)/T05(1,4))-180;
else
    theta1 = atand(T05(2,4)/T05(1,4));
end
t1 = (real(theta1+180));

%% Info
fprintf("end-angle: %4.2f\n", final_x_angle);
fprintf("I_Theta1: %4.2f ", real(theta1));
fprintf("I_Theta2: %4.2f ", real(theta2));
fprintf("I_Theta3: %4.2f ", real(theta3));
fprintf("I_Theta4: %4.2f\n", real(theta4));
fprintf("\n")

fprintf("t1: %4.2f\n", t1);
fprintf("t2: %4.2f\n", t2);
fprintf("t3: %4.2f\n", t3);
fprintf("t4: %4.2f\n", t4);
fprintf("\n");

end
