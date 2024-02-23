%if motion of motor is not smooth, configure an input position filter
%and/or increase the baud rate
clear;

portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 19200);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate

% a plot to monitor the position of the shaft
% also displays motor temperature and force values
position_plot = figure;
hold on
h = text(0,10,'Temp','FontSize',12);
ylim([0 150]);
num_samples = 200;
x = 1:num_samples;
y = zeros(size(x));
r = zeros(size(x));
iteration = 1;
yyaxis left
p = plot (x,y);
ylim([0 150]);
ylabel('Position (mm)');
%create a second axis for Force display
yyaxis right
p1 = plot (x,r);
ylim([-10 10]);
ylabel('Force (N)');
ytickformat('%.3f')
%yticks(-30:1:10);

% Memory Map register address
MODE_OF_OPERATION_address   = 317;

orca.op_mode = 0;
%Put the motor to sleep to start in a known state and clear any active
%errors
while orca.op_mode ~= orca.SleepMode
    orca.change_mode(orca.SleepMode);
    orca.op_mode = orca.read_register(MODE_OF_OPERATION_address, 1);
end
 
%Sine wave parameters
tic;
freq = 0.5;
Amplitude = 30000;
Offset = 50000;

% Loop while streaming a sine wave position target to the motor
%plot motor's position, and force output, display temperature
while true
    t = toc;
    orca.command_orca_position(Amplitude * sin(t*2*pi*freq)+ Offset);
        % Create a 'walking' plot of the data
    iteration = iteration + 1;
    if iteration <= num_samples
        y(iteration) = double(orca.position)/1000.;
        r(iteration) = double(orca.force)/1000.;
    else
        y = circshift(y, -1);
         r = circshift(r, -1);
        y(end) = double(orca.position)/1000.;
        r(end) = double(orca.force)/1000.;
    end
    set(p, 'XData',x, 'YData', y)
    set(p1, 'XData',x, 'YData', r)
    drawnow
    h.String = sprintf('Temp:%d', orca.temperature);
end