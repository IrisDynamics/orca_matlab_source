clear;

ERROR_0_address = 432; % register contains the active errors of the motor, full list of register addresses can be found in the Orca memory map

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 921600);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate, can be configured in IrisControls Modbus page

ERROR_0_value = 0;
% Loop while reading the error register and break if errors are encountered
while ERROR_0_value == 0
    ERROR_0_value = orca.read_register(ERROR_0_address, 1); % Read the value from the ERROR_0 register
end
fprintf("Program Exited due to Motor Error: %d", ERROR_0_value);