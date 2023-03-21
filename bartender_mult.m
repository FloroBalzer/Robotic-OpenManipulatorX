function bartender(shot_no ,port_num, protocol_version)
%shots is the number of shots it will pour
shots = [];
shot1_pos = [18, 8 ];
shots = [shots;shot1_pos];
shot2_pos = [18,4];
shots = [shots;shot2_pos];
shot3_pos = [18,0];
shots = [shots; shot3_pos];
shot4_pos = [18,-4];
shots = [shots;shot4_pos];
shot5_pos = [18,-8];
shots = [shots;shot5_pos];

distance = 5;
bottle_pos = [-1,-13];
dgrid = 2.5;
handheight = 3.5*2.5;
liftheight = 8;%how high the bottle gets lifted after pick up
open_angle = 95;
close_angle = 133;
pourheight = 8; 
pourtime=1.5;

%set up  starting postion for bottle
x_grid = bottle_pos(1); y_grid = bottle_pos(2);
bottle = [x_grid*dgrid, y_grid*dgrid];

%bottle grab spot
bottle_grab = [-1.2, -6];
x_grid = bottle_grab(1); y_grid = bottle_grab(2);
bottle_grabm = [x_grid*dgrid, y_grid*dgrid];





%% moving to bottle grab spot
point_matrix = [];

pos1 = [5*2.5, 0, 15, 0];
pos2 = [bottle_grabm(1), bottle_grabm(2), handheight, 0];
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


%% moving to bottle 
point_matrix = [];

pos1 = [bottle_grabm(1), bottle_grabm(2), handheight, 0];
pos2 = [bottle(1), bottle(2), handheight, 0];
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
pause(0.3)
%% close gripper
pause(1)
write4ByteTxRx(port_num, protocol_version, 15, 116, close_angle/0.088);
pause(1)


%% lift up bottle
point_matrix = [];

pos1 = [bottle(1), bottle(2), handheight, 0];
pos2 = [bottle(1), bottle(2), handheight+liftheight, 0];
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

%% move to shot
    point_matrix = [];

    pos1 = [bottle(1), bottle(2), handheight+liftheight, 0];
    pos2 = [shot1_pos(1), shot1_pos(2), handheight+liftheight, 0];
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


    
    

shot_count = shot_no;
j=1;
while j <= 5
    %% pour
    point_matrix = [];

    pos1 = [shots(j,1), shots(j,2), handheight+liftheight, 0];
    pos2 = [shots(j,1), shots(j,2), pourheight, 1];
    point_matrix = [point_matrix; pos1, pos2, 15, 1, 0, 0, 0];
    [trajectory2, no_point1] = trajectory_path(point_matrix);

    i = 1;
    while i < no_point1
        write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory2(i,1)/0.088);
        write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory2(i,2)/0.088);
        write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory2(i,3)/0.088);
        write4ByteTxRx(port_num, protocol_version, 14, 116, (trajectory2(i,4)+30)/0.088);
        i =i+1;
        pause(0.2)
    end

    pause(pourtime)
    %% unpour
    point_matrix = [];

    pos1 = [shots(j,1), shots(j,2), pourheight, 1];
    pos2 = [shots(j,1), shots(j,2), handheight+liftheight, 0];
    point_matrix = [point_matrix; pos1, pos2, 15, 1, 0, 0, 0];
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
   
    %% next shot
    point_matrix = [];

    pos1 = [shots(j,1), shots(j,2), handheight+liftheight, 0];
    pos2 = [shots(j,1), shots(j,2), handheight+liftheight, 0];
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
 j = j+1;
end

%% move bottle back
point_matrix = [];

pos1 = [shot1_pos(1), shot1_pos(2), handheight+liftheight, 0];
pos2 = [bottle(1), bottle(2), handheight+liftheight, 0];
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
pause(0.5)
%% lower bottle
point_matrix = [];

pos1 = [bottle(1), bottle(2), handheight+liftheight, 0];
pos2 = [bottle(1), bottle(2), handheight, 0];
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
%% leave bottle area
point_matrix = [];

pos1 = [bottle(1), bottle(2), handheight, 0];
pos2 = [bottle_grabm(1), bottle_grabm(2), handheight, 0];
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




