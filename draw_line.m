function draw_line(start_pos,end_pos,num_points, port_num, protocol_version)
%% setting up constants
draw_height = 6.2;
start = [start_pos, draw_height, 1/sqrt(2)];
endp = [end_pos, draw_height, 1/sqrt(2)];
[trajectory1] = linetasktrajectory(start,endp,num_points);

%% drawing line
i = 1;
while i < no_point1
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory1(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory1(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory1(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory1(i,4)/0.088);
    i =i+1;
    pause(0.2)
end

end