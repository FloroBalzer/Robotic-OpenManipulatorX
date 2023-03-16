function set_v_a(velocity, accleration, port_num, protocol_version)
write4ByteTxRx(port_num, protocol_version, 11, 112, velocity);
write4ByteTxRx(port_num, protocol_version, 12, 112, velocity);
write4ByteTxRx(port_num, protocol_version, 13, 112, velocity);
write4ByteTxRx(port_num, protocol_version, 14, 112, velocity);
write4ByteTxRx(port_num, protocol_version, 15, 112, velocity);
write4ByteTxRx(port_num, protocol_version, 11, 108, accleration);
write4ByteTxRx(port_num, protocol_version, 12, 108, accleration);
write4ByteTxRx(port_num, protocol_version, 13, 108, accleration);
write4ByteTxRx(port_num, protocol_version, 14, 108, accleration);
write4ByteTxRx(port_num, protocol_version, 15, 108, accleration);
end