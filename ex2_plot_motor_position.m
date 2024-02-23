clear;


portnum = inputdlg('Enter RS422 COM Port Number:'); % Enter the com port number used by the RS422 cable
port = strcat("COM", portnum);

orca = Actuator(port, 19200);    %COM port of RS422 Modbus channel to Orca Series Motor and default Modbus baud rate

% a plot to monitor the position of the shaft
position_plot = figure;
hold on
ylim([0 150]);
num_samples = 200;
x = 1:num_samples;
y = zeros(size(x));
p = plot (x,y);
iteration = 1;
ylabel('Position (mm)');

% Loop while commanding zero force to motor and plotting position
while true
   orca.command_orca_force(0);  %command 0 newtons to the motor

    % Create a 'walking' plot of the data
    iteration = iteration + 1;
    if iteration <= num_samples
        y(iteration) = double(orca.position)/1000.;
    else
        y = circshift(y, -1);
        y(end) = double(orca.position)/1000.;
    end
    set(p, 'XData',x, 'YData', y)
    drawnow
end