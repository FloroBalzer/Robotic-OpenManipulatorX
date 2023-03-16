function execute_trajectory(trajectory, no_point, port_num, protocol_version)
i=1
while( i< no_point)
    write4ByteTxRx(port_num, protocol_version, 11, 116, trajectory(i,1)/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, trajectory(i,2)/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, trajectory(i,3)/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, trajectory(i,4)/0.088);
    write4ByteTxRx(port_num, protocol_version, 15, 116, 180/0.088);
    i = i+1;
end
end