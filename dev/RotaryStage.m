classdef RotaryStage
    %EPGRIPPER Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        serialPort
        position
    end
    
    methods
        function obj = RotaryStage()
            % Establish the serial port communication
            display('[RotaryStage] Opening Serial Port on COM3')
            obj.serialPort = serial('COM3');
            set(obj.serialPort,'BaudRate',9600,'DataBits',8,'StopBits',1,'Parity','none');
            fopen(obj.serialPort);
            obj.serialPort.Terminator = 'CR';
            fprintf(obj.serialPort, 'HR');
            fprintf(obj.serialPort, 'AC5');
            fprintf(obj.serialPort, 'DE5');
            fprintf(obj.serialPort, 'VE5');
            display('[RotaryStage] Opened Serial Port on COM3')
            
        end
        
        function moveToAngle(obj, val)
            % val is the angle to move to
            STEPS_PER_DEG = 4000;
            l_steps = STEPS_PER_DEG*val;
            l_DI = strcat('DI',num2str(l_steps));
            fprintf(obj.serialPort, l_DI);
            fprintf(obj.serialPort, 'FL');
            readData = fscanf(obj.serialPort);
            display(readData);
        end
        
        
        function delete(obj)
            %DELETE Disconnects from the curvature controller
            %   The embedded curvature controller is actually implemented as a SIMULINK
            %   simulation running on another Windows PC machine. The communication
            %   with the controller is establisher via a serial link with the below
            %   properties.
            
            % Close the serial port communication
            fclose(obj.serialPort);
            display('[EPMgripper] Closed Serial Port on COM3')
            delete(obj.serialPort);
            display('[EPMgripper] Deleted Serial Port on COM3')
        end
        
    end
    
end

