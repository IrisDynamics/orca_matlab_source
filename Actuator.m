classdef Actuator < handle
    %ACTUATOR Summary of this class goes here
    %   Detailed explanation goes here
    properties (Constant)
        %% Modbus Function Codes
        Read = 3
        Write = 6
        Write_Multi = 16
        MotorCommand = 100
        MotorRead = 104
        MotorWrite = 105
        %% Modes of Operation
        SleepMode       = 1
        ForceMode       = 2
        PositionMode    = 3
        HapticMode      = 4
        KinematicMode   = 5
        %% Haptic Effect Flags
        ConstF          = 0b1
        Spring0         = 0b10
        Spring1         = 0b100
        Spring2         = 0b1000
        Damper          = 0b10000
        Inertia         = 0b100000
        Osc0            = 0b1000000
        Osc1            = 0b10000000

        %% Control Registers
        CTRL_REG_0 = 0
        CTRL_REG_1 = 1
        CTRL_REG_2 = 2
        CTRL_REG_3 = 3
    end

    properties
        %% Serial Port Configuration
        comport; 
        baudrate;
        s; 

        %% Orca motor data
        position = 0;
        force = 0;
        power = 0;
        temperature = 0;
        voltage = 0;
        errors = 0;
        op_mode = 0;

    end

    
    methods
        %% ACTUATOR Construct an instance of this class
        % specify the com port and baud rate for modbus communications to
        % the motor. Ex Actuator("COM5", 19200)
        function obj = Actuator(com_port, baud_rate)
            clear obj.s;
            obj.comport = com_port;
            obj.baudrate = baud_rate;
            obj.s = serialport(com_port, baud_rate, "Parity", "even");
        end
       
        %% Command the Orca motor with a specified force in milliNewtons, receive the current sensed position (micrometers) as output
        %Uses the motor command stream function code to send target force positions. Returns position and updates object�s parameters (position, force, power, temperature, voltage, errors)
        function position = command_orca_force(obj,force_mN)            
            forcebytes = int32(typecast(int32(force_mN),'uint8'));
            obj.s.write( obj.append_crc([1, obj.MotorCommand, 28, forcebytes(4), forcebytes(3), forcebytes(2), forcebytes(1) ]), "uint8");
            data = obj.s.read(19, "uint8");
            %address = data(1);
            %func_code = data(2);
            obj.position = typecast (uint8([data(6) data(5) data(4) data(3)]), 'int32');
            obj.force = typecast (uint8([data(10) data(9) data(8) data(7)]), 'int32');
            obj.power =  typecast (uint8([data(12) data(11)]), 'uint16');
            obj.temperature = data(13);
            obj.voltage =  typecast (uint8([data(15) data(14)]), 'uint16');
            obj.errors =  typecast (uint8([data(17) data(16)]), 'uint16');
            position = double(obj.position);                                %casting here is for purpose of simulink block
            if (obj.errors == 2048)
                change_mode(obj, 1);
            end
        end

        %% Command the Orca motor to a specified position(micrometers), receive the current sensed force (millinewtons) as output
        %Uses the motor command stream function code to send target position values. (position, force, power, temperature, voltage, errors)
        function force = command_orca_position(obj,position_um)
            positionbytes = int32(typecast(int32(position_um),'uint8'));
            obj.s.write( obj.append_crc([1, obj.MotorCommand, 30, positionbytes(4), positionbytes(3), positionbytes(2),positionbytes(1)]), "uint8");
            data = obj.s.read(19, "uint8");
            
            %address = data(1);
            %func_code = data(2);
            obj.position = typecast (uint8([data(6) data(5) data(4) data(3)]), 'int32');
            obj.force = typecast (uint8([data(10) data(9) data(8) data(7)]), 'int32');
            obj.power =  typecast (uint8([data(12) data(11)]), 'uint16');
            obj.temperature = data(13);
            obj.voltage =  typecast (uint8([data(15) data(14)]), 'uint16');
            obj.errors =  typecast (uint8([data(17) data(16)]), 'uint16');
            force = double(obj.force);                                      %casting here is for purpose of simulink block
            if (obj.errors == 2048)
                change_mode(obj, 1);
            end
        end
        
        %% Read Stream function code 
        %Uses the motor read stream function code to read from a specified register (16 or 32 bit) while also updating object�s parameters (position, force, power, temperature, voltage, errors)
        function read_value = read_stream(obj, register_address, width)
             addressbytes = obj.u16_to_bytes(register_address);
             obj.s.write( obj.append_crc([1, obj.MotorRead,addressbytes, width]), "uint8");
             data = obj.s.read(24, "uint8");
             %address = data(1);
             %func_code = data(2);
             read_value = typecast (uint8([data(6) data(5) data(4) data(3)]), 'int32');
             obj.op_mode = data(7);
             obj.position = typecast (uint8([data(11) data(10) data(9) data(8)]), 'int32');
             obj.force = typecast (uint8([data(15) data(14) data(13) data(12)]), 'int32');
             obj.power =  typecast (uint8([data(17) data(16)]), 'uint16');
             obj.temperature = data(18);
             obj.voltage =  typecast (uint8([data(20) data(19)]), 'uint16');
             obj.errors =  typecast (uint8([data(22) data(21)]), 'uint16');
             read_value = double(read_value);
        end
        
        %% Write stream function code
        %Uses the motor write stream function code to write to a specified register while also updating object�s parameters (position, force, power, temperature, voltage, errors)
        function force = write_stream(obj, registeraddr, width, value)
             addressbytes = obj.u16_to_bytes(registeraddr);
             valuebytes = int32(typecast(int32(value),'uint8'));
             obj.s.write( obj.append_crc([1, obj.MotorWrite, addressbytes, width, valuebytes(4), valuebytes(3), valuebytes(2), valuebytes(1)]), "uint8");
             data = obj.s.read(20, "uint8");
             %address = data(1);
             %func_code = data(2);
             obj.op_mode = data(3);
             obj.position = typecast (uint8([data(7) data(6) data(5) data(4)]), 'int32');
             obj.force = typecast (uint8([data(11) data(10) data(9) data(8)]), 'int32');
             obj.power =  typecast (uint8([data(13) data(12)]), 'uint16');
             obj.temperature = data(14);
             obj.voltage =  typecast (uint8([data(16) data(15)]), 'uint16');
             obj.errors =  typecast (uint8([data(18) data(17)]), 'uint16');
             force = double(obj.force);
        end
        
        %% Change Orca's mode of operation
        %Write to control register 3 to change the Orca�s mode of operation (Sleep, Force, Position, Kinematic, Haptic)
        function change_mode(obj,mode)
            obj.write_register(3, mode);        %CTRL_REG_3 3 (mode change register)
        end        

        %% Trigger Kinematic Motion
        % Write to the kinematic software trigger register the ID of the motion that will be triggered (motor must be in kinematic mode to have an effect)
        function kinematic_trigger(obj, motionID)
            obj.write_register(9, motionID);   %KIN_SW_TRIGGER 9
        end

        %% Configure Kinematic Motion
        %Write to multiple registers that will configure a specified motion ID with all relevant motion parameters.
        function configure_motion(obj, motionID, position, time, delay, nextID, type, autonext)
            positionL_H = obj.int32_to_u16(position);
            timeL_H = obj.int32_to_u16(time);
            next_type_auto = bitshift(nextID, 3) + bitshift(type, 1) + autonext;
            configuration = [positionL_H, timeL_H, delay,next_type_auto];
            obj.write_multi_registers(780 + motionID*6, 6, configuration);  % KIN_MOTION_0 780
        end

        %% Enable Specified haptic effects
        % Turn on and off various combinations of haptic effects
        function enable_haptic_effect(obj, effect_bits)
            obj.write_register(641, effect_bits);    %HAPTIC_STATUS 641
        end 

        %% Configure Haptic Effect Spring A
        % Configure all parameters relevant to spring A�s haptic behaviour. 
        %This is meant as an example of adding additional methods to the Actuator class.
        function configure_springA(obj, gain, center, coupling, deadzone, force_sat)
            center = typecast(int32(center), 'uint16');
            configuration = [gain center coupling deadzone force_sat];
            obj.write_multi_registers(644, 6, configuration);  %write to spring A configurations
        end

        %% Tune PID values
        %A custome function can be added for specific use cases
        % From the Orca memory map (in the orca reference
        %manual), the starting address for pid tuning is found to be 133 (PC_PGAIN) to
        %have a single modbus message write to multiple register, the
        %registers must all be consecutive, so if we want to write to only
        %PC_PGAIN and PC_FSTAU_H, also registers in between would need to
        %be set as well (this is likely still faster than sending 2
        %messagse to write).
        %the first thing to do is to create an array of all the register
        %values, and some values that are 32 bit values and take 2
        %registers (for example saturation) need to be split into 2, 16
        %bit values. 
        %Set the tuning of the motor%s PID position controller. 
        %This controller will be active in Position and Kinematic modes.

        function tune_pid_controller(obj, saturation, p_gain, i_gain, dv_gain, de_gain)
            saturationL_H = obj.int32_to_u16(saturation); %split 32 bit into 2 16 bit, low first hi second
            configuration = [p_gain, i_gain, dv_gain, de_gain , saturationL_H];
            obj.write_multi_registers(133, 6, configuration);  % PC_PGAIN 133, num consecutive registers 6, register values  configuration
        end
        

        %% Write Single Registers
        %Write a value to a single register
        function write_register(obj, register_address, value)
            addressbytes = obj.u16_to_bytes(register_address);
            valuebytes = obj.u16_to_bytes(value);
            obj.s.write(obj.append_crc([1, obj.Write, addressbytes, valuebytes]), "uint8");  
            obj.s.read( 8, "uint8");
        end

        %% Write Multiple Registers
        %Write an array of data to a series of consecutive registers.
        % data should be an array of values for each registers and will be
        % converted to bytes to send in this function
        function write_multi_registers(obj, registers_start_address, num_registers, register_data)
            register_data = uint16(register_data);
            databytes = obj.u16_to_bytes(register_data);
            num_registersbytes = obj.u16_to_bytes(num_registers);
            addressbytes = obj.u16_to_bytes(registers_start_address);
            obj.s.write(obj.append_crc([1, obj.Write_Multi,  addressbytes, num_registersbytes, num_registers*2, databytes]), "uint8"); 
            obj.s.read( 8, "uint8");
        end

        %% Read Registers
        function read_value = read_register(obj, registers_start_address, num_registers)
            addressbytes = obj.u16_to_bytes(registers_start_address);
            num_registersbytes = obj.u16_to_bytes(num_registers);
            obj.s.write(obj.append_crc([1, obj.Read,  addressbytes, num_registersbytes]), "uint8");  
            data = obj.s.read(5 + 2*num_registers, "uint8");
            read_value = zeros(1, num_registers);
            for i = 1:num_registers
                read_value(i) = typecast (uint8([data(2*(i-1) + 5) data(2*(i-1) + 4)]), 'uint16');
            end
        end


    end

    methods (Static)
        %% Append CRC bytes to a message 
        % Appends the crc (Low byte, high byte) to message for modbus
        % communication. Message is an array of bytes.
        function amsg = append_crc(message)
            N = length(message);
            crc = hex2dec('ffff');
            polynomial = hex2dec('a001');
            for i = 1:N
                crc = bitxor(crc,message(i));
                for j = 1:8
                    if bitand(crc,1)
                        crc = bitshift(crc,-1);
                        crc = bitxor(crc,polynomial);
                    else
                        crc = bitshift(crc,-1);
                    end
                end
            end
            lowByte = bitand(crc,hex2dec('ff'));
            highByte = bitshift(bitand(crc,hex2dec('ff00')),-8);
            amsg = message;
            amsg(N+1) = lowByte;
            amsg(N+2) = highByte;
        end
        
        %% U16 to bytes
        % Converts an unsigned 16 bit piece of data (register value) into
        % an array of bytes in the correct order for modbus message parsing
        function bytes = u16_to_bytes(data)
            bytes = int32(typecast(swapbytes(uint16(data)), 'uint8'));
        end

        function u16 = int32_to_u16(data)
            u16 = int32(typecast(int32(data), 'uint16'));
        end

    end
end





