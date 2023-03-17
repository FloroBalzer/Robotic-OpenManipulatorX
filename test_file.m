%% Setting up
%point_matrix = [point_matrix; start_pos, end_pos, subdivsion count, line or circle, center_pos];

point_matrix = [];

%start here
start_pos = [20, 0, 0, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [0, 20, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];

start_pos = [0, 20, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [-20, 0, 20, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [-20, 0, 20, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [0, -20, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [0, -20, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [20, 0, 0, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [20, 0, 0, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [20, 0, 20, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
point_matrix = [point_matrix; start_pos, end_pos, 10, 1, 0, 0, 0];


start_pos = [20, 0, 20, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [0, -20, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [0, -20, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [-20, 0, 0, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [-20, 0, 0, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [0, 20, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];


start_pos = [0, 20, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [20, 0, 20, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [0, 0, 10];
point_matrix = [point_matrix; start_pos, end_pos, 10, 2, center_pos];

%end here

[trajectory, no_point] = trajectory_path(point_matrix);
[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(1,1), trajectory(1,2), trajectory(1,3), trajectory(1,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [x, y, z];

%% Running Simulation
i=1;
while i <= no_point 

[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4))



[robot_body] = Simulation(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4), x, y, z, T05(1,4),T05(2,4), T05(3,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [Endposition; x, y, z];

pause(0.2)
delete(robot_body);
i = i+1;
end

% disp(Endposition);
% disp(trajectory);
