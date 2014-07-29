classdef Arm2D < handle
    %ARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties 
        theta0 = pi/2;           % is the current/measured initial orientation of the first segment
        L = [];                  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        above_max = 'false'      % boolean if any segments are above k limits 
        k_measured               % measured arc curvatures 
        curvature_controller     % curvature controller        
    end
    
    properties(Dependent)
        k_target                 % target arc curvatures 
    end
    
    properties(Constant)
        N = 6;                   % is the total number of arm segments
        k_min = -20;             % minimum allowable curvature
        k_max = 20;              % maximum allowable curvature
    end
    
    methods

        function set.k_target(obj, val)
           dims = size(val);
           if( dims(1) == obj.N && dims(2) == 1)
               for i = 1:obj.N
                  if( val(i) < obj.k_min || val(i) > obj.k_max )
                      obj.above_max = 'true';
                  end
               end
               
               if( obj.above_max ~= 'true' )
                    obj.k_target = val;
               end
               %send_curvature_errors(obj.curvature_controller, obj.k_target', obj.k_measured)
           else
               error('The size of k does not match the arm.')
           end
        end
        
        function obj = setMeasured(obj, k, L)
           obj.k_measured = k;
           obj.L = L;
        end
        
        function command = send_curvature_errors( controller, target, measured )
        %SEND_CURVATURE_ERRORS Sends error to the embedded PID curvature controller
        %   INPUT:
        %   controller	Serial Port Object - curvature controller object
        %   target              1xS vector - target curvatures
        %   measured            1xS vector - measured curvatures

        function [ k ] = select_curvature(target, measured)
        %   Selects the appropriate curvature control bits. If the measured
        %   curvature is above a safe hard-coded threshold, we are safe in
        %   actuating using the motor opposite to the current curvature.
        %   
        %   However if we are in a transition region close to 0 curvature, we are
        %   not sure what the actual curvature of the segment is. To be safe, we
        %   will actuate the segment with the motor opposite to the target
        %   curvature.
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

        % Threshold and multiplier. The threshold corresponds to a curvature of
        % 5 m^-1, or alternatively a radius of curvature of 20 cm. The theory is
        % that curvature above the value of 5 will be noise-immune.
        % 
        % The multiplier c will give a curvature difference of 40 the maximum value
        % of error at +127. Curvature of 40 m^-1 corresponds to a radius of
        % curvature of 2.5 cm, which is around an inch. This will roughly mean that
        % angle between the beginning and end of a segment is 90 degrees.
        % NOTE: This does not limit the angle by which the segment can bend to 90
        % degrees, but makes the error saturate at roughly 90 degrees, what is a
        % sane choice. So if a larger curvature was requested and possible, it will
        % still be driven to that value.
        threshold = inf;
        c = 127/40;

        % Prepare the command packet. K is the vector of curvatures of segment
        % which effectively chooses which motor to use to control the position. The
        % (measured-target) error is multiplied by (-1).^(K-1) because of the way
        % the motors expect the sign of errors to be supplied.
        K = arrayfun(@(t,m) select_curvature(t,m), target, measured);
        E = c * (target-measured);
        command = [K; E];

        % Expand the command packed along its rows. This is the format which is
        % used by serial port to communicate the error command.
        command = int8( reshape(command,1,[]) );

        % Send the command
        fwrite(controller, command, 'int8');

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

    end
    
end

