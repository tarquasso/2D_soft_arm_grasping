classdef Arm2D < handle
    %ARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        theta0 = pi/2;           % is the current/measured initial orientation of the first segment
        L = [];                  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        k_measured               % measured arc curvatures
        k_target                 % target arc curvatures
        curvatureController      % curvature controller
        armDims                  % Arm dimensions
        gripper;                 % Gripper2D
    end
       
    properties(Constant)
        N = 6;                   % is the total number of arm segments
        k_min = -20;             % minimum allowable curvature
        k_max = 20;              % maximum allowable curvature
    end
    
    methods
        
        function obj = Arm2D()
            % Dimensions and orientation of the arm
            obj.armDims = struct();
            obj.armDims.S = 6;
            obj.armDims.lengths = repmat(2.37,1,obj.armDims.S) .* ...
                unitsratio('m','inch');
            obj.armDims.spos = [263.8; 152.7] .* unitsratio('m','mm');
            obj.armDims.srot= pi/2;
            
            %Create a curvature controller
            obj.curvatureController = CurvatureController;
            
            
        end
        
        function set.k_target(obj, val)
           dims = size(val);
           if( dims(1) == obj.N && dims(2) == 1)
              
               above_max = 'false';
               
               for i = 1:obj.N
                  if( (val(i) < obj.k_min) || (val(i) > obj.k_max) )
                      above_max = 'true';
                  end
               end

               if( strcmp(above_max, 'false') )
                   obj.k_target = val;
                   %send_curvature_errors(obj.curvature_controller, obj.k_target', obj.k_measured);
               else
                   error('An element of k exceeds allowable limit')
               end
               
           else
               error('The size of k does not match the arm.')
           end
        end
        
        function set.k_measured(obj, val)
           dims = size(val);
           if( dims(1) == obj.N && dims(2) == 1)
              obj.k_measured = val; 
           else
              error('The size of measured k does not match the arm.');
           end
        end
        
        function set.L(obj, val)
           dims = size(val);
           if( dims(1) == obj.N && dims(2) == 1)
              obj.L = val; 
           else
              error('The size of measured L does not match the arm.');
           end
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
        
        function [x, y, theta] = recursive_forward_kinematics(obj, gripper_on, k, i, s)

           % gripper_on boolean if gripper attached
           % k - either measured or target curvature vector
           % i is the segment of interest
           % s is the length of interest along segment i
           
           if ( gripper)
           N = obj.N;
           theta0 = obj.theta0;
           L = obj.L;
            
           dims = size(k);
           if( dims(1) ~= obj.N || dims(2) ~= 1)
              error('The size of measured k does not match the arm.');
           end
        
            theta_init = zeros(1,N);
            x_init = zeros(1,N);
            y_init = zeros(1,N);

            if( i == 1)
                theta_init(i) = theta0;
                x_init(i) = 0.0;
                y_init(i) = 0.0;
            else
                [x_init(i), y_init(i), theta_init(i)] = obj.recursive_forward_kinematics(obj, k, i-1, L(i-1));
            end

            theta = theta_init(i) + k(i)*s;
            x = x_init(i) + sin(theta)/k(i) - sin(theta_init(i))/k(i);
            y = y_init(i) - cos(theta)/k(i) + cos(theta_init(i))/k(i);
            
        end
        
        function h = draw(obj) 
            
            theta0 = obj.theta0;
            N = obj.N + 1;
            L = [obj.L; obj.gripper.L];
            k = [obj.k_measured; obj.gripper.k];

            M = 20;
            total = 1;
            x = zeros(1, N*M);
            y = zeros(1, N*M);
            theta = zeros(1, N*M);

            for i=1:N
                for j=1:M
                    [x(total), y(total), theta(total)] = recursive_forward_kinematics(k, theta0, L, N, i, L(i)*(j/M));
                    total = total + 1;
                end
            end

            hold on
            axis([-0.30 0.30 -0.10 0.50])
            axis square

            h = plot(x(1:end-20),y(1:end-20), 'r', x(end-20:end),y(end-20:end), 'k', 'LineWidth', 2);
            
        end

    end
    
end

