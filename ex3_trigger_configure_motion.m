clear;

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 19200);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate

% a plot to monitor the position of the shaft
position_plot = figure;
hold on
h = text(10,10,'Active ID','FontSize',14);
ylim([0 150]);
num_samples = 200;
x = 1:num_samples;
y = zeros(size(x));
p = plot (x,y);
iteration = 1;
ylabel('Position (mm)');

%Memory Map addresses of registers to be read
KIN_STATUS_address          = 319;
MODE_OF_OPERATION_address   = 317;

%Put the motor to sleep to start in a known state and clear any active
%errors
orca.op_mode = 0;
while orca.op_mode ~= orca.SleepMode
    orca.change_mode(orca.SleepMode);
    orca.op_mode = orca.read_register(MODE_OF_OPERATION_address, 1);
end

%configure a set of kinematic motions with the final motion automatically
%looping to the first
orca.configure_motion(0, 50000, 1000, 200, 1, 0, 0);
orca.configure_motion(1, 80000,  800,  20, 2, 0, 0);
orca.configure_motion(2, 50000,  500,   0, 3, 0, 0);
orca.configure_motion(3, 20000,  300,   0, 0, 0, 1);

%Ensure the motor gets into kinematic mode. This will immediately trigger
%the home motion.
while orca.op_mode ~= orca.KinematicMode
    orca.change_mode(orca.KinematicMode);
    orca.op_mode = orca.read_register(MODE_OF_OPERATION_address, 1);
end

% Loop while triggering kinematic motions, plotting position and displaying
% currently active motion id
while true
    %kinematic status gives the running bit and the active ID
    kin_status = orca.read_stream(KIN_STATUS_address, 1);  %read the current state of the kinematic controller, while also updating all motor data

    active_id = bitand(kin_status, 0x7FFF);
    kin_running = bitand(kin_status, 0x8000);
    %When active motion is done running trigger next motion.
    if(kin_running ==0)
        orca.kinematic_trigger(active_id + 1);
    end
   
    % Create a 'walking' plot of the position data
    iteration = iteration + 1;
    if iteration <= num_samples
        y(iteration) = orca.position/1000.;
    else
        y = circshift(y, -1);
        y(end) = orca.position/1000.;
    end
    set(p, 'XData',x, 'YData', y)
    drawnow
    h.String = sprintf('Active ID:%d', active_id); %Update text on the figure with active motion IDs
end