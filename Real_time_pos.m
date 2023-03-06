% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates
clc;
clear all;
lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% ---- Control Table Addresses ---- %%
%Control table of EEPROM Area
addr_pro_model_number      = 0;
addr_pro_model_information = 2;
addr_pro_firmware_version  = 6;
addr_pro_id                = 7;
addr_pro_baud_rate         = 8;
addr_pro_return_delay_time = 9;
addr_pro_drive_mode        = 10;
addr_pro_operating_mode    = 11;
addr_pro_secondary_id      = 12;
addr_pro_protocol_type     = 13;
addr_pro_homing_offset     = 20;
addr_pro_moving_threshold  = 24;
addr_pro_temperature_limit = 31;
addr_pro_max_voltage_limit = 32;
addr_pro_min_voltage_limit = 34;
addr_pro_pwm_limit         = 36;
addr_pro_current_limit     = 38;
addr_pro_velocity_limit    = 44;
addr_pro_max_position      = 48;
addr_pro_min_position      = 52;
addr_pro_startup_config    = 60;
addr_pro_shutdown          = 63;

%Control table of RAM Area
addr_pro_torque_enable          = 64;
addr_pro_LED                    = 65;
addr_pro_status_return_level    = 68;
addr_pro_registered_instruction = 69;
addr_pro_hardware_error_status  = 70;
addr_pro_velocity_I_gain        = 76;
addr_pro_velocity_P_gain        = 78;
addr_pro_velocity_D_gain        = 80;
addr_pro_position_I_gain        = 82;
addr_pro_position_P_gain        = 84;
addr_pro_feedforward_2_gain     = 88;
addr_pro_feedforward_1_gain     = 90;
addr_pro_bus_watchdog           = 98;
addr_pro_goal_pwm               = 100;
addr_pro_goal_current           = 102;
addr_pro_goal_velocity          = 104;
addr_pro_profile_acceleration   = 108;
addr_pro_profile_velocity       = 112;
addr_pro_goal_position          = 116;
addr_pro_realtime_tick          = 120;
addr_pro_moving                 = 122;
addr_pro_moving_status          = 123;
addr_pro_present_pwm            = 124;
addr_pro_present_current        = 126;
addr_pro_present_velocity       = 128;
addr_pro_present_position       = 132;
addr_pro_velocity_trajectory    = 136;
addr_pro_position_trajectory    = 140;
addr_pro_present_input_voltage  = 144;
addr_pro_present_temperature    = 146;

%% ---- Other Settings ---- %%

% Protocol version
protocol_version            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
dxl_11                      = 11;            % Dynamixel ID: 1
dxl_12                      = 12;            % Dynamixel ID: 1
dxl_13                      = 13;            % Dynamixel ID: 1
dxl_14                      = 14;            % Dynamixel ID: 1
dxl_15                      = 15;            % Dynamixel ID: 1

baudrate                     = 115200;
device_name                  = 'COM10';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
torque_enable               = 1;            % Value for enabling the torque
torque_disable              = 0;            % Value for disabling the torque
dxl_11_min_pos_val  = -150000;      
dxl_11_max_pos_val  = 150000;       
dxl_moving_status_threshold = 20;           % Dynamixel moving status threshold

%operating modes
curr_control_mode           = 0;
velo_control_mode           = 1;
pos_control_mode            = 2;
ext_pos_control_mode        = 3;
curr_pos_control_mode       = 4;
pwm_control_mode            = 16;

esc_character               = 'e';          % Key for escaping loop

comm_success                = 0;            % Communication Success result value
comm_tx_fail                = -1001;        % Communication Tx Failed
%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(device_name);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = comm_tx_fail;           % Communication result
dxl_11_goal_position = [dxl_11_min_pos_val dxl_11_max_pos_val];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end


% Set port baudrate
if (setBaudRate(port_num, baudrate))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end
%% ---------Start--------- %%
% Put actuator into Position Control Mode
write1ByteTxRx(port_num, protocol_version, dxl_11, addr_pro_operating_mode, pos_control_mode);
write1ByteTxRx(port_num, protocol_version, dxl_12, addr_pro_operating_mode, pos_control_mode);
write1ByteTxRx(port_num, protocol_version, dxl_13, addr_pro_operating_mode, pos_control_mode);
write1ByteTxRx(port_num, protocol_version, dxl_14, addr_pro_operating_mode, pos_control_mode);
write1ByteTxRx(port_num, protocol_version, dxl_15, addr_pro_operating_mode, pos_control_mode);

%% ---------Read_Position_code--------- %%
i = 0;
while(i<8)
    dxl_11_present_position = read4ByteTxRx(port_num, protocol_version, dxl_11, addr_pro_present_position);
    dxl_12_present_position = read4ByteTxRx(port_num, protocol_version, dxl_12, addr_pro_present_position);
    dxl_13_present_position = read4ByteTxRx(port_num, protocol_version, dxl_13, addr_pro_present_position);
    dxl_14_present_position = read4ByteTxRx(port_num, protocol_version, dxl_14, addr_pro_present_position);
    dxl_15_present_position = read4ByteTxRx(port_num, protocol_version, dxl_15, addr_pro_present_position);

    dxl_11_present_angle = dxl_11_present_position*0.088; %needs offset
    dxl_12_present_angle = dxl_12_present_position*0.088; %needs offset
    dxl_13_present_angle = dxl_13_present_position*0.088; %needs offset
    dxl_14_present_angle = dxl_14_present_position*0.088; %needs offset
    dxl_15_present_angle = dxl_15_present_position*0.088; %needs offset
        
    fprintf('[ID:%03d] Position11: %03d\n', dxl_11, typecast(uint32(dxl_11_present_position), 'int32'));
    fprintf('[ID:%03d] Angle11: %d\n', dxl_11, dxl_11_present_angle);
    fprintf('[ID:%03d] Position12: %03d\n', dxl_12, typecast(uint32(dxl_12_present_position), 'int32'));
    fprintf('[ID:%03d] Angle12: %d\n', dxl_12, dxl_12_present_angle);
    fprintf('[ID:%03d] Position13: %03d\n', dxl_13, typecast(uint32(dxl_13_present_position), 'int32'));
    fprintf('[ID:%03d] Angle13: %d\n', dxl_13, dxl_13_present_angle);
    fprintf('[ID:%03d] Position14: %03d\n', dxl_14, typecast(uint32(dxl_14_present_position), 'int32'));
    fprintf('[ID:%03d] Angle14: %d\n', dxl_14, dxl_14_present_angle);
    fprintf('[ID:%03d] Position15: %03d\n', dxl_15, typecast(uint32(dxl_15_present_position), 'int32'));   
    fprintf('[ID:%03d] Angle15: %d\n', dxl_15, dxl_15_present_angle);
    
    Simulation(dxl_11_present_angle, dxl_12_present_angle, dxl_13_present_angle, dxl_14_present_angle);
        
    pause(0.5)
    i=i+1;
end

%% ---------Power_off--------- %%
% Disable Dynamixel Torque
write1ByteTxRx(port_num, protocol_version, dxl_11, addr_pro_torque_enable, torque_disable);
write1ByteTxRx(port_num, protocol_version, dxl_12, addr_pro_torque_enable, torque_disable);
write1ByteTxRx(port_num, protocol_version, dxl_13, addr_pro_torque_enable, torque_disable);
write1ByteTxRx(port_num, protocol_version, dxl_14, addr_pro_torque_enable, torque_disable);
write1ByteTxRx(port_num, protocol_version, dxl_15, addr_pro_torque_enable, torque_disable);
dxl_comm_result = getLastTxRxResult(port_num, protocol_version);
dxl_error = getLastRxPacketError(port_num, protocol_version);

if dxl_comm_result ~= comm_success
    fprintf('%s\n', getTxRxResult(protocol_version, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(protocol_version, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;
