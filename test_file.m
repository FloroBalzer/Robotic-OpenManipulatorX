%% Setting up
start_pos = [10, 0, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [10, -10, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
no_point1 = 10;
trajectory1 = linetasktrajectory(start_pos, end_pos, no_point1);

start_pos = [10, -10, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [20, 0, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
no_point2 = 10;
trajectory2 = linetasktrajectory(start_pos, end_pos, no_point2);

start_pos = [20, 0, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [20, 10, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
no_point3 = 10;
trajectory3 = linetasktrajectory(start_pos, end_pos, no_point3);

start_pos = [20, 10, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [10, 10, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
no_point4 = 10;
trajectory4 = linetasktrajectory(start_pos, end_pos, no_point4);

start_pos = [10, 10, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [15, 5, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [10, 5, 10];
no_point5 = 10;
trajectory5 = circletasktrajectory(start_pos, end_pos,center_pos, no_point5);

start_pos = [15, 5, 10, 1/sqrt(2)];% format [x, y, z, sin(angle)]
end_pos = [10, 0, 10, 1/sqrt(2)]; % format [x, y, z, sin(angle)]
center_pos = [10, 5, 10];
no_point6 = 10;
trajectory6 = circletasktrajectory(start_pos, end_pos,center_pos, no_point6);


trajectory = [trajectory1; trajectory2; trajectory3; trajectory4; trajectory5; trajectory6];
[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(1,1), trajectory(1,2), trajectory(1,3), trajectory(1,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [x, y, z];



%% Running Simulation
i=1;
while i <= (no_point1+no_point2+no_point3+no_point4+no_point5+no_point6) 

delete(robot_body);
[T01, T12, T23, T34, T45, T02, T03, T04, T05] = ForwardKinematics(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4))

[robot_body] = Simulation(trajectory(i,1), trajectory(i,2), trajectory(i,3), trajectory(i,4), x, y, z, T05(1,4),T05(2,4), T05(3,4));
x = T05(1,4); y = T05(2,4); z = T05(3,4);
Endposition = [Endposition; x, y, z];


pause(0.1)
i = i+1;
end

disp(Endposition);
disp(trajectory);
