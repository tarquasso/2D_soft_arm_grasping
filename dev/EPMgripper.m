classdef EPMgripper
    %EPGRIPPER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        serialPort
    end
    
    methods
        function obj = EPMgripper()
            % Establish the serial port communication
            display('[EPMgripper] Opening Serial Port on COM10')
            obj.serialPort = serial('COM10');
            set(obj.serialPort,'BaudRate',37600,'DataBits',8,'StopBits',1,'Parity','none');
            fopen(obj.serialPort);
            display('[EPMgripper] Opened Serial Port on COM10')
            
        end
        
        function gripperOn(obj)
            % Send the command
            fwrite(obj.serialPort, 'n', 'int8'); 
            readData = fscanf(obj.serialPort);
            display(readData);
            pause(0.10);
            fwrite(obj.serialPort, '1', 'int8'); 
            readData = fscanf(obj.serialPort);
            display(readData);
            pause(0.10);
            
        end
        
        function gripperOff(obj)
            % Send the command
            fwrite(obj.serialPort, 'f', 'int8');
            readData = fscanf(obj.serialPort);
            display(readData);
            pause(0.10);
            fwrite(obj.serialPort, '1', 'int8');
            readData = fscanf(obj.serialPort);
            display(readData);
            pause(0.10);
            
        end
        
        function delete(obj)
            %DELETE Disconnects from the curvature controller
            %   The embedded curvature controller is actually implemented as a SIMULINK
            %   simulation running on another Windows PC machine. The communication
            %   with the controller is establisher via a serial link with the below
            %   properties.
            
            % Close the serial port communication
            fclose(obj.serialPort);
            display('[EPMgripper] Closed Serial Port on COM10')
            delete(obj.serialPort);
            display('[EPMgripper] Deleted Serial Port on COM10')
        end
        
    end
    
end

