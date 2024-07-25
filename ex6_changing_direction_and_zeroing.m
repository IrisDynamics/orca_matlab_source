clear;

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 912600);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate

orca.set_direction(1); %Sets direction
orca.configure_zero(2,30,1); %Sets autozero to Auto Zero Enabled, force to 30N, and exit mode to sleep
orca.trigger_zero(); %Triggers the zero routine


