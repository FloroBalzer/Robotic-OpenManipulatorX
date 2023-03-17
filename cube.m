 function cube(cube_start, cube_position, cube_end, rotation_count, rotation_position, stack_no, port_num, protocol_version)

%% set up constants
116 =116
;cube_height = 2.5;
move_height = 10;
height_offset = stack_no * cube_height;
dgrid = 2.5;
offset = dgrid;
open_angle = 120;
close_angle = 230;

x_rotlimit = 1;                                                 %figure out
y_rotlimit = 1;                                                 %figure out
rot_tracker = rotation_count;


%set up  starting postion
x_grid = cube_start(1); y_grid = cube_start(2);
start_pos = [x_grid*dgrid, y_grid*dgrid];

%set up cube postion
x_grid2 = cube_position(1); y_grid2 = cube_position(2);
cube_pos = [x_grid2*dgrid, y_grid2*dgrid];

%set up cube end position
x_grid3 = cube_end(1); y_grid3 = cube_end(2);
cube_endpos = [x_grid3*dgrid, y_grid3*dgrid];

%if need, set up rotation position
x_grid4 = rotation_position(1); y_grid4 = rotation_position(2);                                   %change as necessary
rotation_pos = [x_grid4*dgrid, y_grid4*dgrid];


%% moving to cube
point_matrix = [];

pos1 = [start_pos, start_pos, move_height, 1];
pos2 = [cube_pos(1), cube_pos(2), move_height, 1];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
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

%% open gripper
pause(1)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(1)

%% lower to pick up cube 
point_matrix = [];

pos1 = [cube_pos(1), cube_pos(2), move_height, 1];
pos2 = [cube_pos(1), cube_pos(2), cube_height, 1];
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

%% close gripper
pause(1)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(1)

%% raise cube after grip 
point_matrix = [];

pos1 = [cube_pos(1), cube_pos(2), cube_height, 1];
pos2 = [cube_pos(1), cube_pos(2), move_height, 1];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
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

temp_pos = cube_pos;

%% rotation
while rot_tracker ~= 0
    if(cube_pos(1) < x_rotlimit && cube_pos(2) < y_rotlimit) %within limit area, move to rotation location
        point_matrix = [];
        %move to rotation spot
        pos1 = [cube_pos(1), cube_pos(2), move_height, 1];% 
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %rotate on the spot
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, 1];% 
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %lowering cube
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, 0];
        pos2 = [rotation_pos(1), rotation_pos(2), cube_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %open gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
        pause(1)
        %move up
        pos1 = [rotation_pos(1), rotation_pos(2), cube_height, 0];
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %rotate
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, 0];% 
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %move down
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, 1];% 
        pos2 = [rotation_pos(1), rotation_pos(2), cube_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(1)
        %close gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
        pause(1)
        %move up
        pos1 = [rotation_pos(1), rotation_pos(2), cube_height, 1];% 
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(1)
        
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
        
        temp_pos = rotation_pos;
    
    else
        %rotate on the spot
        point_matrix = [];
        pos1 = [cube_pos(1), cube_pos(2), move_height, 1];% 
        pos2 = [cube_pos(1), cube_pos(2), move_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %lowering cube
        pos1 = [cube_pos(1), cube_pos(2), move_height, 0];
        pos2 = [cube_pos(1), cube_pos(2), cube_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %open gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
        pause(1)
        %move up
        pos1 = [cube_pos(1), cube_pos(2), cube_height, 0];
        pos2 = [cube_pos(1), cube_pos(2), move_height, 0];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %rotate
        pos1 = [cube_pos(1), cube_pos(2), move_height, 0];% 
        pos2 = [cube_pos(1), cube_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
        pause(1)
        %move down
        pos1 = [cube_pos(1), cube_pos(2), move_height, 1];% 
        pos2 = [cube_pos(1), cube_pos(2), cube_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(1)
        %close gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
        pause(1)
        %move up
        pos1 = [cube_pos(1), cube_pos(2), cube_height, 1];% 
        pos2 = [cube_pos(1), cube_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(1)
        
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
    
    rot_tracker = rot_tracker - 1;
end

%% move to end position
point_matrix = [];

pos1 = [temp_pos(1), temp_pos(2), move_height, 1];% format [x, y, z, sin(angle)]
pos2 = [cube_endpos(1), cube_endpos(2), move_height, 1];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
[trajectory5, no_point5] = trajectory_path(point_matrix);

i = 1;
while i < no_point5
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory5(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory5(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory5(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory5(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

%% lower to release
point_matrix = [];

pos1 = [cube_endpos(1), cube_endpos(2), move_height, 1];
pos2 = [cube_endpos(1), cube_endpos(2), cube_height + height_offset, 1];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
[trajectory6, no_point6] = trajectory_path(point_matrix);

i = 1;
while i < no_point6
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory6(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory6(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory6(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory6(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

%% open gripper
pause(1)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(1)

%% raise to finish sequence
point_matrix = [];

pos1 = [cube_endpos(1), cube_endpos(2), cube_height + height_offset, 1];
pos2 = [cube_endpos(1), cube_endpos(2), move_height, 1];
point_matrix = [point_matrix; pos1, pos2, 5, 1, 0, 0, 0];
[trajectory7, no_point7] = trajectory_path(point_matrix);

i = 1;
while i < no_point7
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory7(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory7(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory7(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory7(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

end