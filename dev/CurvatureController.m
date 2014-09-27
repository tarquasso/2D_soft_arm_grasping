classdef CurvatureController < handle
    %CURVATURECONTROLLER Communicating with Simulink Controller on PC2
    % The embedded curvature controller is actually implemented as a SIMULINK
    % simulation running on another Windows PC machine. The communication
    % with the controller is establisher via a serial link with the below
    % properties.
    
    properties
        serialPort
        vectorLength
        expType
    end
    
    methods
        function obj = CurvatureController(vecLength,expType)
            if nargin < 2
                obj.expType = ExpTypes.PhysicalExperiment;
            else
                if(isa(expType,'ExpTypes'))
                    obj.expType = expType;
                else
                    error('wrong type');
                end
            end
            obj.vectorLength = vecLength;
            if (obj.expType == ExpTypes.PhysicalExperiment || obj.expType == ExpTypes.Tuning )           
                % Establish the serial port communication
                display('[CurvatureController] Opening Serial Port on COM1')
                obj.serialPort = serial('COM1');
                set(obj.serialPort,'BaudRate',57600,'DataBits',8,'StopBits',1,'Parity','none');
                fopen(obj.serialPort);
                display('[CurvatureController] Opened Serial Port on COM1')
            end
        end
        
        function command = sendCurvatureErrors( obj, target, measured )
            %SEND_CURVATURE_ERRORS Sends error to the embedded PID curvature controller
            %   INPUT:
            %   obj         Serial Port Object - curvature controller object
            %   target      1xS vector - target curvatures
            %   measured    1xS vector - measured curvatures
            
            %error checking
            l_targetSize = size(target);
            if( l_targetSize(2) ~= obj.vectorLength && l_targetSize(1) == 1)
                error('sendCurvatureErrors function input target has wrong size');
            end
            
            l_measuredSize = size(measured);
            if( l_measuredSize(2) ~= obj.vectorLength && l_measuredSize(1) == 1)
                error('sendCurvatureErrors function input measured has wrong size');
            end
            
            % The multiplier c will give a curvature difference of 40 the maximum value
            % of error at +127. Curvature of 40 m^-1 corresponds to a radius of
            % curvature of 2.5 cm, which is around an inch. This will roughly mean that
            % angle between the beginning and end of a segment is 90 degrees.
            % NOTE: This does not limit the angle by which the segment can bend to 90
            % degrees, but makes the error saturate at roughly 90 degrees, what is a
            % sane choice. So if a larger curvature was requested and possible, it will
            % still be driven to that value.
            c = 127/20;
            
            % Prepare the command packet. K is the vector of curvatures of
            % segment which effectively chooses which motor to use to control
            % the position. The (measured-target) error is multiplied by
            % (-1).^(K-1) because of the way the motors expect the sign of
            % errors to be supplied.
            
            K = arrayfun(@(t,m) CurvatureController.select_curvature(t,m), ...
                target, measured);
            E = c * (target-measured);
            %TODO: adjust for the l;arger k of the gripper
            E(1,end)= 127/50*(target(1,end)-25.0); %this is a hack for now
            command = [K; E];
            
            % Expand the command packed along its rows. This is the format which is
            % used by serial port to communicate the error command.
            command = int8( reshape(command,1,[]) );
            if(obj.expType == ExpTypes.PhysicalExperiment || obj.expType == ExpTypes.Tuning)
            % Send the command
            fwrite(obj.serialPort, command, 'int8');
            elseif(obj.expType == ExpTypes.Simulation)
            %TODO: plot or log command for simuolation
            end
            % Output the input and output data.
            % fprintf('mes: ');
            % fprintf('%9.2f',measured);
            %
            % fprintf('\ntgt: ');
            % fprintf('%9.2f',target);
            %
            % fprintf('\nk:   ');
            % fprintf('%9d',K);
            %
            % fprintf('\nerr: ');
            % fprintf('%9d', int8(E));
            % fprintf('\n\n')
    
        end
        
        function delete(obj)
            %DELETE Disconnects from the curvature controller
            %   The embedded curvature controller is actually implemented as a SIMULINK
            %   simulation running on another Windows PC machine. The communication
            %   with the controller is establisher via a serial link with the below
            %   properties.
            if (obj.expType == ExpTypes.PhysicalExperiment || obj.expType == ExpTypes.Tuning)
                % Close the serial port communication
                fclose(obj.serialPort);
                display('[CurvatureController] Closed Serial Port on COM1')
                delete(obj.serialPort);
                display('[CurvatureController] Deleted Serial Port on COM1')
            end
        end   
    end
    
    
    methods(Static)
        function [ k ] = select_curvature(target, measured)
            %   Selects the appropriate curvature control bits. If the measured
            %   curvature is above a safe hard-coded threshold, we are safe in
            %   actuating using the motor opposite to the current curvature.
            %
            %   However if we are in a transition region close to 0 curvature, we are
            %   not sure what the actual curvature of the segment is. To be safe, we
            %   will actuate the segment with the motor opposite to the target
            %   curvature.
            
            % The threshold corresponds to a curvature of
            % 5 m^-1, or alternatively a radius of curvature of 20 cm. The theory is
            % that curvature above the value of 5 will be noise-immune.
            threshold = 5;
            
            if abs(measured) >= threshold
                if measured >= 0.0
                    k = 1;
                else
                    k = -1;
                end
            else
                if target >= 0.0
                    k = 1; %1
                else
                    k = -1; %-1
                end
            end
        end
    end
    
end

