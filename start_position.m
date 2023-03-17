function start_position(port_num, protocol_version)
    write4ByteTxRx(port_num, protocol_version, 11, 116, 180/0.088);
    write4ByteTxRx(port_num, protocol_version, 12, 116, 100/0.088);
    write4ByteTxRx(port_num, protocol_version, 13, 116, 210/0.088);
    write4ByteTxRx(port_num, protocol_version, 14, 116, 230/0.088);
    write4ByteTxRx(port_num, protocol_version, 15, 116, 230/0.088); %always check close gripper position first
end