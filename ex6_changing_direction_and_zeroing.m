clear;

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 19200);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate


orca.set_direction(1); %Sets direction (0 or 1). Do not change the direrction while the motor is running

orca.configure_zero(2,30,1); %Sets autozero to Auto Zero Enabled, force to 30N, and exit mode to sleep

orca.auto_zero_wait(); %Triggers the zero routine. While not continue until zeroing is complete


disp("Motor has zeroed")