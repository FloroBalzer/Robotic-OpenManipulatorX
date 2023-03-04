%Plotting joint axes
grid on

xlim([-0.2, 0.2]);
ylim([-0.2, 0.2]); 
zlim([0, 0.2]);
view(135, 45);

[T01, T12, T23, T34,T02, T03, T04, T05] = ForwardKinematics(50, 50, 50, 50);

% % Origin
origin = [0 0 0];
X0 =[0.01 0 0];
Y0 =[0 0.01 0];
Z0 =[0 0 0.01];

line('Xdata', X0,'Ydata', origin, 'ZData', origin, 'LineWidth', 1, 'Color', 'g'); %base x-axis
line('Xdata', origin,'Ydata', Y0, 'ZData', origin, 'LineWidth', 1, 'Color', 'b'); %base y-axis
line('Xdata', origin,'Ydata', origin, 'ZData', Z0, 'LineWidth', 1, 'Color', 'r'); %base z-axis

base = [0; 0; 0; 1];
% % Joint 1
n_origin = transpose(T01*base);
n_origin = n_origin(1:end-1);
disp(n_origin)

line('Xdata', X0,'Ydata', n_origin, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'g'); %Joint 1 x-axis
line('Xdata', n_origin,'Ydata', Y0, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'b'); %Joint 1 y-axis
line('Xdata', n_origin,'Ydata', n_origin, 'ZData', Z0, 'LineWidth', 1, 'Color', 'r'); %Joint 1 z-axis

% % Joint 2
n_origin = transpose(T02*base);
n_origin = n_origin(1:end-1);
disp(n_origin)

line('Xdata', X0,'Ydata', n_origin, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'g'); %Joint 2 x-axis
line('Xdata', n_origin,'Ydata', Y0, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'b'); %Joint 2 y-axis
line('Xdata', n_origin,'Ydata', n_origin, 'ZData', Z1, 'LineWidth', 1, 'Color', 'r'); %Joint 2 z-axis

% % Joint 3
n_origin = transpose(T03*base);
n_origin = n_origin(1:end-1);
disp(n_origin)

line('Xdata', X0,'Ydata', n_origin, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'g'); %Joint 3 x-axis
line('Xdata', n_origin,'Ydata', Y0, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'b'); %Joint 3 y-axis
line('Xdata', n_origin,'Ydata', n_origin, 'ZData', Z1, 'LineWidth', 1, 'Color', 'r'); %Joint 3 z-axis

% % Joint 4
n_origin = transpose(T04*base);
n_origin = n_origin(1:end-1);
disp(n_origin)

line('Xdata', X0,'Ydata', n_origin, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'g'); %Joint 4 x-axis
line('Xdata', n_origin,'Ydata', Y0, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'b'); %Joint 4 y-axis
line('Xdata', n_origin,'Ydata', n_origin, 'ZData', Z1, 'LineWidth', 1, 'Color', 'r'); %Joint 4 z-axis

% % Joint 5
n_origin = transpose(T05*base);
n_origin = n_origin(1:end-1);
disp(n_origin)

line('Xdata', X0,'Ydata', n_origin, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'g'); %Joint 5 x-axis
line('Xdata', n_origin,'Ydata', Y0, 'ZData', n_origin, 'LineWidth', 1, 'Color', 'b'); %Joint 5 y-axis
line('Xdata', n_origin,'Ydata', n_origin, 'ZData', Z1, 'LineWidth', 1, 'Color', 'r'); %Joint 5 z-axis
