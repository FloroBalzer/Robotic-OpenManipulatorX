%% Setting up
%point_matrix = [point_matrix; start_pos, end_pos, subdivsion count, line or circle, center_pos];

point_matrix = [];

%start here
%start set up for cube grab
start_pos = [5, 0, 5, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [5, 0, 20, 0.3]; % format [x, y, z, sin(angle)]
point_matrix = [point_matrix; start_pos, end_pos, 5, 1, 0, 0, 0];

start_pos = [5, 0, 20, 0.3];% format [x, y, z, sin(angle)]
end_pos = [8, 0, 8, 1]; % format [x, y, z, sin(angle)]
point_matrix = [point_matrix; start_pos, end_pos, 5, 1, 0, 0, 0]; %maintain height above cubes at 8 when possible
%end set up for cube grab

%start move to cube position
dgrid = 2.5;
x_grid = 6;
y_grid = 4;
start_pos = [8, 0, 8, 1];% format [x, y, z, sin(angle)]
end_pos = [dgrid*x_grid, dgrid*y_grid, 8, 1]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 5, 1, 0, 0, 0];
%end move to cube position


%lower arm to grab, open gripper first
start_pos = [dgrid*x_grid, dgrid*y_grid, 8, 1];% format [x, y, z, sin(angle)]
end_pos = [dgrid*x_grid, dgrid*y_grid, 3, 1]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 5, 1, 0, 0, 0];
%close gripper at end of sequence

%move up, change 4th value to 0 to rotate
start_pos = [dgrid*x_grid, dgrid*y_grid, 3, 1];% format [x, y, z, sin(angle)]
end_pos = [dgrid*(x_grid+1), dgrid*(y_grid+1), 15, 0]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 10, 1, 0, 0, 0];
%end move up

%lower back down
start_pos = [dgrid*(x_grid+1), dgrid*(y_grid+1), 15, 0];% format [x, y, z, sin(angle)]
end_pos = [dgrid*x_grid, dgrid*y_grid, 3, 0]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 10, 1, 0, 0, 0];
%open gripper

%go up and rotate
start_pos = [dgrid*x_grid, dgrid*y_grid, 3, 0];% format [x, y, z, sin(angle)]
end_pos = [dgrid*x_grid, dgrid*y_grid, 8, 1]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 10, 1, 0, 0, 0];

%return to home
start_pos = [dgrid*x_grid, dgrid*y_grid, 8, 1];% format [x, y, z, sin(angle)]
end_pos = [8, 0, 8, 1]; % format [x, y, z, sin(angle)] %edit x and y to cube position note that holes are at 2.5 grid
point_matrix = [point_matrix; start_pos, end_pos, 5, 1, 0, 0, 0];

%end here

[trajectory, no_point] = trajectory_path(point_matrix);
[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(1,1), trajectory(1,2), trajectory(1,3), trajectory(1,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [x, y, z];

%% Running Simulation
i=1;
while i <= no_point 

[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4))

delete(robot_body);

[robot_body] = Simulation(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4), x, y, z, T05(1,4),T05(2,4), T05(3,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [Endposition; x, y, z];

pause(0.2)

i = i+1;
end

% disp(Endposition);
% disp(trajectory);
