function pen_return(start_position, port_num, protocol_version)
%% set up constants
move_height_init = 6;
move_height = 15;
draw_height = 6.5;
open_angle = 120;
close_angle = 226;
pen_pos = [16, -16];
end_position = [5, 5]


%% raise pen
point_matrix = [];

pos1 = [start_position(1), start_position(2), draw_height, 1/sqrt(2)];
pos2 = [start_position(1), start_position(2), move_height, 1/sqrt(2)];
point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
[trajectory1, no_point1] = trajectory_path(point_matrix);

i = 1;
while i < no_point1
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory1(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory1(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory1(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory1(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

%% moving to pen holder
point_matrix = [];

pos1 = [start_position(1), start_position(2), move_height, 1/sqrt(2)];
pos2 = [pen_pos(1), pen_pos(2), move_height, 1/sqrt(2)];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
[trajectory2, no_point2] = trajectory_path(point_matrix);

i = 1;
while i < no_point2
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory2(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory2(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory2(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory2(i,4)/0.088);
    i =i+1;
    pause(0.2)
end


%% lower to pen position
point_matrix = [];

pos1 = [pen_pos(1), pen_pos(2), move_height, 1/sqrt(2)];
pos2 = [pen_pos(1), pen_pos(2), move_height_init, 1/sqrt(2)];
point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
[trajectory3, no_point3] = trajectory_path(point_matrix);

i = 1;
while i < no_point3
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory3(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory3(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory3(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory3(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

%% open gripper
pause(1)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(1)

%% moving to end position
point_matrix = [];

pos1 = [pen_pos(1), pen_pos(2), move_height_init, 1/sqrt(2)];
pos2 = [end_position(1), end_position(2), move_height, 1/sqrt(2)];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
[trajectory4, no_point4] = trajectory_path(point_matrix);

i = 1;
while i < no_point4
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory4(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory4(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory4(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory4(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

end