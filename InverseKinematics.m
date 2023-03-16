function [t1, t2, t3, t4] = InverseKinematics(px, py, pz, sinphi)
%% setup
t0 = acosd((0.130^2+0.128^2-0.024^2)/(2*0.130*0.128));

a0 = 0;      alpha0 = 0;     d1 = 7.7;               % Link 1
a1 = 0;      alpha1 = 90;    d2 = 0;                 % Link 2
a2 = 13.0;   alpha2 = 0;     d3 = 0;                 % Link 3
a3 = 12.4;   alpha3 = 0;     d4 = 0;                 % Link 4
a4 = 12.6;   alpha4 = 0;     d5 = 0;                 % Gripper

%switching endposition to st space
tool_s = sqrt(px.^2 + py.^2);
tool_t = pz-d1;

%% helper formulae
final_x_angle = asind(sinphi);

joint4_s = tool_s - a4 *cosd(final_x_angle);
if final_x_angle < 0
    joint4_t = tool_t + a4 * sind(final_x_angle);
else
    joint4_t = tool_t - a4 * sind(final_x_angle);
end
joint4_t = tool_t + a4 * sind(final_x_angle);
gamma = acosd(((a2.^2+a3.^2)-(joint4_s.^2 + (joint4_t).^2))/(2*a2*a3));
alpha = atan2d(joint4_t,joint4_s);
beta = asind((a3 * sind(gamma))/(sqrt(joint4_s.^2 + joint4_t.^2)));
%% T1
if (px>0 && py>0)
    theta1 = atand(py/px);
elseif (px<0 && py>0)
    theta1 = atand(py/px)+180;
elseif (px<0 && py<0)
    theta1 = atand(py/px)-180;
else
    theta1 = atand(py/px);
end
t1 = (real(theta1+180));

%% T3
theta3 = 180-gamma;
t3 = (real(theta3 + 90 + t0));

%% T2
% if px > -1
    theta2 = alpha + beta;
    t2 = (real(270 - theta2 - t0));%elbow out
% elseif px > -1 && pz < 0
%     theta2 = alpha + beta;
%     t2 = (real(270 - theta2 - 2.5*t0));%elbow out
% else
%     theta2 = alpha - beta;
%     t2 = (real(90 - theta2 + t0));%elbow in
% end
 %% T4
%if px > -1 && pz > 2
    theta4 = final_x_angle -t2-t3;
    t4 = (real(theta4))+540;
%disp(theta2+theta3);

%% T1
if (px>0 && py>0)
    theta1 = atand(py/px);
elseif (px<0 && py>0)
    theta1 = atand(py/px)+180;
elseif (px<0 && py<0)
    theta1 = atand(py/px)-180;
elseif (px<0 && py==0)
    theta1 = -180;
else
    theta1 = atand(py/px);
end
t1 = (real(theta1+180));

%% info and debug
%joint position in st-space
joint4_s = tool_s - a4 *cosd(final_x_angle);
joint4_t = tool_t - a4 * sind(final_x_angle);
joint3_s = joint4_s - a3 * cosd(theta2+theta3);
joint3_t = joint4_t - a3 * sind(theta2+theta3);
joint2_s = joint3_s - a2 * cosd(theta2);
joint2_t = joint3_t - a2 * sind(theta2);

% fprintf("t1: %4.2f\n", t1);
% fprintf("t2: %4.2f\n", t2);
% fprintf("t3: %4.2f\n", t3);
% fprintf("t4: %4.2f\n", t4);
% fprintf("\n");
% % 
% fprintf("alpha: %4.2f\n", alpha);
% fprintf("beta: %4.2f\n", beta);
% fprintf("gamma: %4.2f\n", gamma);
% fprintf("end-angle: %4.2f\n", final_x_angle);
% fprintf("I_Theta1: %4.2f ", real(theta1));
% fprintf("I_Theta2: %4.2f ", real(theta2));
% fprintf("I_Theta3: %4.2f ", real(theta3));
% fprintf("I_Theta4: %4.2f\n", real(theta4));
% fprintf("\n")
% 
% fprintf("tool_s: %4.3f ", tool_s);
% fprintf("tool_t: %4.3f\n", tool_t);
% fprintf("\n")
% 
% fprintf("joint4_s: %4.3f ", joint4_s);
% fprintf("joint4_t: %4.3f\n", joint4_t);
% fprintf("joint_4 length: %4.2f\n", sqrt((tool_s-joint4_s).^2+(tool_t-joint4_t).^2))
% fprintf("\n")
% 
% fprintf("joint3_s: %4.3f ", joint3_s);
% fprintf("joint3_t: %4.3f\n", joint3_t);
% fprintf("joint_3 length: %4.2f\n", sqrt((joint4_s-joint3_s).^2+(joint4_t-joint3_t).^2))
% fprintf("\n")
% 
% fprintf("joint2_s: %4.3f ", joint2_s);
% fprintf("joint2_t: %4.3f\n", joint2_t);
% fprintf("joint_2 length: %4.2f\n", sqrt((joint3_s - joint2_s).^2+(joint3_t - joint2_t).^2))
% fprintf("\n")



    
end
