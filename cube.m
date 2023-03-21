 function cube(cube_start, cube_position, cube_end, rotation_count, rotation_position, stack_no, port_num, protocol_version)

%% set up constants
cube_height = 3.6;
grab_height = 4.1;
move_height = 10;
height_offset = stack_no * cube_height-1;  
dgrid = 2.5;
offset =0;
open_angle = 91.5;
close_angle = 208;

%distance at which robot cannot rotate the cube on the spot
x_rotlimit = 15;                                                 %figure out
y_rotlimit = 15;                                                 %figure out
%distance at which gripper orientation cannot be 90 degrees down
x_angle_limit = 17; 
y_angle_limit = 17;
too_far = 0;
gripper_angle = 1; %given in sine of angle
rot_tracker = rotation_count;


%set up  starting postion
x_grid = cube_start(1)*dgrid; y_grid = cube_start(2)*dgrid;
start_pos = [x_grid, y_grid];

%set up cube postion
x_grid2 = cube_position(1)*dgrid; y_grid2 = cube_position(2)*dgrid;
cube_pos = [x_grid2, y_grid2];

%set up cube end position
x_grid3 = cube_end(1)*dgrid; y_grid3 = cube_end(2)*dgrid;
cube_endpos = [x_grid3, y_grid3];

%if need, set up rotation position
x_grid4 = rotation_position(1)*dgrid; y_grid4 = rotation_position(2)*dgrid;
rotation_pos = [x_grid4, y_grid4];

if (abs(cube_pos(1)) > 7 && abs(cube_pos(2)) > 7)
    innit_height = 5;

% elseif (abs(cube_pos(1)) < 15 && abs(cube_pos(2)) < 15)
%     innit_height = 0;
else
    innit_height = cube_height-1;
end



%set gripper angle condition
if sqrt(cube_pos(1)^2 + cube_pos(2)^2) < sqrt(x_angle_limit^2 + y_angle_limit^2)
    gripper_angle = 1;
else
    gripper_angle = 1/sqrt(2);
    too_far = 1;
end



%% moving to cube
point_matrix = [];

pos1 = [start_pos(1), start_pos(2), 15, gripper_angle];
pos2 = [cube_pos(1), cube_pos(2), move_height, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
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
pause(0.2)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(0.2)

%% lower to pick up cube 
point_matrix = [];

pos1 = [cube_pos(1), cube_pos(2), move_height, gripper_angle];
pos2 = [cube_pos(1), cube_pos(2), innit_height, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
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
pause(0.2)
write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
pause(0.2)

%% raise cube after grip 
point_matrix = [];

pos1 = [cube_pos(1), cube_pos(2), cube_height, gripper_angle];
pos2 = [cube_pos(1), cube_pos(2), move_height, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
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
    
    point_matrix = [];
    %move to rotation spot
    pos1 = [cube_pos(1), cube_pos(2), move_height, gripper_angle];% 
    pos2 = [rotation_pos(1), rotation_pos(2), move_height, gripper_angle];
    point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
    pause(0.2)
    if too_far ==1
        %lowering cube
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, gripper_angle];
        pos2 = [rotation_pos(1), rotation_pos(2), grab_height, gripper_angle];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(0.2)
        
        [trajectory4a, no_point4a] = trajectory_path(point_matrix);
        i = 1;
        while i < no_point4a
            write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory4a(i,1)/0.088);
            write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory4a(i,2)/0.088);
            write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory4a(i,3)/0.088);
            write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory4a(i,4)/0.088);
            i =i+1;
            pause(0.2)
        end
        %open gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
        pause(0.5)
        
        point_matrix = [];            
        %move up
        point_matrix = [];
        pos1 = [rotation_pos(1), rotation_pos(2), grab_height, gripper_angle];
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, gripper_angle];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(0.2)
        %rotate
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, gripper_angle];% 
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 15, 1, 0, 0, 0];
        pause(0.2)
        %move down
        pos1 = [rotation_pos(1), rotation_pos(2), move_height, 1];% 
        pos2 = [rotation_pos(1), rotation_pos(2), grab_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
        pause(0.2)
        
        [trajectory4b, no_point4b] = trajectory_path(point_matrix);
        i = 1;
        while i < no_point4b
            write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory4b(i,1)/0.088);
            write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory4b(i,2)/0.088);
            write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory4b(i,3)/0.088);
            write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory4b(i,4)/0.088);
            i =i+1;
            pause(0.2)
        end
        %close gripper
        write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
        pause(0.2)
        
        point_matrix = [];
        %move up
        pos1 = [rotation_pos(1), rotation_pos(2), grab_height, 1];
        pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
        point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
    end
    
    %rotate on the spot
    pos1 = [rotation_pos(1), rotation_pos(2), move_height, gripper_angle];% 
    pos2 = [rotation_pos(1), rotation_pos(2), move_height, 0];
    point_matrix = [point_matrix; pos1, pos2, 15, 1, 0, 0, 0];
    pause(0.2)
    %lowering cube
    pos1 = [rotation_pos(1), rotation_pos(2), move_height, 0];
    pos2 = [rotation_pos(1), rotation_pos(2), grab_height-1, 0];
    point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
    pause(0.2)
    
    [trajectory4c, no_point4c] = trajectory_path(point_matrix);
    i = 1;
    while i < no_point4c
        write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory4c(i,1)/0.088);
        write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory4c(i,2)/0.088);
        write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory4c(i,3)/0.088);
        write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory4c(i,4)/0.088);
        i =i+1;
        pause(0.2)
    end
    
    %open gripper
    write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
    pause(0.2)
    
    point_matrix = [];
    %move up
    pos1 = [rotation_pos(1), rotation_pos(2), grab_height-1, 0];
    pos2 = [rotation_pos(1), rotation_pos(2), move_height, 0];
    point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
    pause(0.2)
    %rotate
    pos1 = [rotation_pos(1), rotation_pos(2), move_height, 0];% 
    pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
    point_matrix = [point_matrix; pos1, pos2, 15, 1, 0, 0, 0];
    pause(0.2)
    %move down
    pos1 = [rotation_pos(1), rotation_pos(2), move_height, 1];% 
    pos2 = [rotation_pos(1), rotation_pos(2), grab_height, 1];
    point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
    pause(0.2)
    
    [trajectory4d, no_point4d] = trajectory_path(point_matrix);
    i = 1;
    while i < no_point4d
        write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory4d(i,1)/0.088);
        write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory4d(i,2)/0.088);
        write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory4d(i,3)/0.088);
        write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory4d(i,4)/0.088);
        i =i+1;
        pause(0.25)
    end
   
    %close gripper
    write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
    pause(0.2)
    
    point_matrix = [];
    %move up
    pos1 = [rotation_pos(1), rotation_pos(2), grab_height, 1];% 
    pos2 = [rotation_pos(1), rotation_pos(2), move_height, 1];
    point_matrix = [point_matrix; pos1, pos2, 10, 1, 0, 0, 0];
    pause(0.2)
    
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
    
    rot_tracker = rot_tracker - 1;
    gripper_angle = 1;
end


%% move to end position

point_matrix = [];

pos1 = [temp_pos(1), temp_pos(2), move_height, gripper_angle];% format [x, y, z, sin(angle)]
pos2 = [cube_endpos(1), cube_endpos(2), move_height, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
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

pos1 = [cube_endpos(1), cube_endpos(2), move_height, gripper_angle];
pos2 = [cube_endpos(1), cube_endpos(2), height_offset, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
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
pause(0.2)
write4ByteTxRx(port_num, protocol_version, 15, 116, open_angle/0.088);
pause(0.2)

%% raise to finish sequence
point_matrix = [];

pos1 = [cube_endpos(1), cube_endpos(2), height_offset, gripper_angle];
pos2 = [cube_endpos(1), cube_endpos(2), move_height, gripper_angle];
point_matrix = [point_matrix; pos1, pos2, 8, 1, 0, 0, 0];
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