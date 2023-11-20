clear;

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 19200);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate
%orca = Actuator(port, 1250000);

%Memory Map addresses of registers
MODE_OF_OPERATION_address   = 317;
S0_GAIN_N_MM_address        = 644;
O0_GAIN_N_address           = 664;
HAPTIC_STATUS_address       = 641;

%% configure spring effect
gain = 150;
center = typecast(int32(65000), 'uint16');
coupling = 0;
deadzone = 0;
force_sat = 0;
configuration = [gain center coupling deadzone force_sat];
orca.write_multi_registers(S0_GAIN_N_MM_address, 6, configuration);

%% configure oscillation effect
 gain = 10; % Newtons
 type = 1; %sine wave
 freq = 1000; %dHz
 duty = 0;
 configuration = [gain type freq duty];
 
 orca.write_multi_registers(O0_GAIN_N_address, 4, configuration); 


orca.enable_haptic_effect (orca.Spring0);

while read_register(orca, MODE_OF_OPERATION_address, 1) ~= orca.SleepMode
    change_mode(orca, orca.SleepMode);          %put the motor to sleep, this will also clear errors
end

while read_register(orca, MODE_OF_OPERATION_address, 1) ~= orca.HapticMode
    change_mode(orca, orca.HapticMode);          %put the motor into haptic mode to start effects
end
%stream a sine wave position target to the motor
%last_position = 0;
quiet_zone = 50000;
haptic_status = 0;
while true
    % set a vibration zone and a quiet zone and update depending on motor
    % position.
    if ((orca.position <= quiet_zone) && (haptic_status ~= orca.Spring0))
        orca.enable_haptic_effect (orca.Spring0 );
    elseif ((orca.position > quiet_zone) && (haptic_status ~= (orca.Spring0 + orca.Osc0)))
        orca.enable_haptic_effect (orca.Spring0 + orca.Osc0);
    end
    %this is to maintain a stream of data coming from the motor ie, position, force, temp etc.
    haptic_status = orca.read_stream(HAPTIC_STATUS_address, 1);                            
end
