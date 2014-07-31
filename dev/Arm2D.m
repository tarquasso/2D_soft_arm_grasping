classdef Arm2D < handle
    %ARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dims                     % Arm dimensions
        gripper2D;               % Gripper2D
        curvatureController      % curvature controller
    end
    properties(SetAccess=private,GetAccess=public)
        L = [];                  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        kMeasured               % measured arc curvatures
        kTarget                % target arc curvatures      
    end
   
    methods
        %Constructor
        function obj = Arm2D()
            % Dimensions and orientation of the arm
            
            obj.dims = struct();
            obj.dims.S = 6;      % is the total number of arm segments
            obj.dims.kMin = -20;     % minimum allowable curvature
            obj.dims.kMax = 20;      % maximum allowable curvature
            obj.dims.theta0 = pi/2;   % is the current/measured initial orientation of the first segment
      
            obj.dims.lengths = repmat(2.37,1,obj.dims.S) .* ...
                unitsratio('m','inch');
            obj.dims.spos = [263.8; 152.7] .* unitsratio('m','mm');
            obj.dims.srot = pi/2;
            
            %Create gripper2D before curvature controller
            obj.gripper2D = Gripper2D;
            %Create a curvature controller
            obj.curvatureController = CurvatureController;

        end
        %Destructor
        function delete(obj)
            obj.gripper2D.delete();
            obj.curvatureController.delete();
        end
        %Set Target Curvatures of the 2D arm
        function setTargetCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(1) == obj.dims.S && l_valSize(2) == 1)
                
                above_max = 'false';
                
                for i = 1:obj.dims.S
                    if( (val(i) < obj.dims.kMin) || (val(i) > obj.dims.kMax) )
                        above_max = 'true';
                    end
                end
                
                if( strcmp(above_max, 'false') )
                    obj.kTarget = val;
                    obj.curvatureController.sendCurvatureErrors(...
                        obj.kTarget', obj.kMeasured);
                else
                    error('An element of k exceeds allowable limit')
                end
                
            else
                error('The size of k does not match the arm.')
            end
        end
        
        function setMeasuredCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(1) == obj.N && l_valSize(2) == 1)
                obj.kMeasured = val;
            else
                error('The size of measured k does not match the arm.');
            end
        end
        
        function setMeasuredLengths(obj, val)
            l_valSize = size(val);
            if( l_valSize(1) == obj.N && l_valSize(2) == 1)
                obj.L = val;
            else
                error('The size of measured L does not match the arm.');
            end
        end
        
        function [x, y, theta] = recursiveForwardKinematics(obj, gripper_on, k, i, s)
            
            % gripper_on - boolean if gripper2D attached
            % k - either measured or target curvature vector
            % i - is the segment of interest
            % s - is the length of interest along segment i
            
            l_valSize = size(k);
            if( l_valSize(1) ~= obj.N || l_valSize(2) ~= 1)
                error('The size of measured k does not match the arm.');
            end
            
            if ( gripper_on )
                l_N = obj.N + 1;
                l_L = [obj.L; obj.gripper2D.L];
                k = [obj.kMeasured; obj.gripper2D.k];
            else
                l_N = obj.N;
                l_L = obj.L;
            end
            
            l_theta0 = obj.theta0;
            
            theta_init = zeros(1,l_N);
            x_init = zeros(1,l_N);
            y_init = zeros(1,l_N);
            
            if( i == 1)
                theta_init(i) = l_theta0;
                x_init(i) = 0.0;
                y_init(i) = 0.0;
            else
                [x_init(i), y_init(i), theta_init(i)] = obj.recursiveForwardKinematics(obj, gripper_on, k, i-1, l_L(i-1));
            end
            
            theta = theta_init(i) + k(i)*s;
            x = x_init(i) + sin(theta)/k(i) - sin(theta_init(i))/k(i);
            y = y_init(i) - cos(theta)/k(i) + cos(theta_init(i))/k(i);
            
        end
        
        function h = plotArmToHandle(obj, gripper_on)
            
            M = 20;
            total = 1;
            x = zeros(1, N*M);
            y = zeros(1, N*M);
            theta = zeros(1, N*M);
            
            for i=1:N
                for j=1:M
                    [x(total), y(total), theta(total)] = ...
                        recursiveForwardKinematics(obj, gripper_on, k, i, ...
                        obj.L(i)*(j/M));
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

