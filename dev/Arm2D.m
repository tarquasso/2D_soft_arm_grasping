classdef Arm2D < handle
    
    properties
        dims                     % Arm dimensions
        numOfRigidBodies         % Number of Rigid Bodies
        gripper2D               % Gripper2D
        curvatureController      % curvature controller
        segPos2D
    end
    
    properties(SetAccess=private,GetAccess=public)
        arcLenMeas               % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        thetaMeas                % angle vector describing rotation of each segment
        kMeas                    % measured arc curvatures
        kTarget                  % target arc curvatures
        
    end
    
    methods
        %Constructor
        function obj = Arm2D( expType )
            
            if nargin < 1
                expType = ExpTypes.PhysicalExperiment;
            else
                if(isa(expType,'ExpTypes'))
                    %obj.expType = expType;
                else
                    error('wrong type');
                end
            end
            
            % Dimensions and orientation of the arm
            obj.dims = struct();
            obj.dims.S = 6;          % is the total number of arm segments
            obj.dims.kMin = [-11.0, -18.5, -11.0, -14.1, -11.8, -17.4];     % minimum allowable curvature
            obj.dims.kMax = [13.8, 10.0, 15.6, 10.3, 16.2, 12.4];      % maximum allowable curvature
            obj.dims.thetaStart = pi/2;   % is the current/measured initial orientation of the first segment
            obj.dims.lengths = repmat(2.47*0.0254,1,obj.dims.S); %[m]
            
            k_init = 0.1*randn(1, 6);
            obj.setMeasuredLengths(2.47*0.0254*ones(1, obj.dims.S));
            obj.setMeasuredCurvatures(k_init);
            obj.setTargetCurvatures(k_init);
    
    
            %Create gripper2D before curvature controller
            obj.gripper2D = Gripper2D;            
    
            obj.numOfRigidBodies = obj.dims.S + obj.gripper2D.dims.S + 1; % plus one is accounting for the base
            %Create a curvature controller
            l_vectorLength = obj.dims.S+obj.gripper2D.dims.S;
            obj.curvatureController = CurvatureController(l_vectorLength, expType);

            
        end
        %Destructor
        function delete(obj)
            obj.gripper2D.delete();
            obj.curvatureController.delete();
        end
        %Set Target Curvatures of the 2D arm
        function setTargetCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                
                above_max = 'false';
                
                for i = 1:obj.dims.S
                    if( (val(i) < obj.dims.kMin(i)) || (val(i) > obj.dims.kMax(i)) )
                        above_max = 'true';
                    end
                end
                
                if( strcmp(above_max, 'false') )
                    obj.kTarget = val;
                else
                    error('An element of k exceeds allowable limit')
                end
                
            else
                error('The size of k does not match the arm.')
            end
        end
        
        function actuate(obj)
            %obj.kTarget - obj.kMeas
            obj.curvatureController.sendCurvatureErrors( [obj.kTarget,obj.gripper2D.kTarget] ,...
                [obj.kMeas,obj.gripper2D.kMeas] );

        end
        %Set Measured Curvatures of the 2D arm
        function setMeasuredCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.kMeas = val;
            else
                error('The size of measured k does not match the arm.');
            end
        end
        %Set Measured Lengths of the 2D arm
        function setMeasuredLengths(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.arcLenMeas = val;
            else
                error('The size of measured arcLenMeas does not match the arm.');
            end
        end
        
        function calculateSegmentValues( obj)
            %SETSEGMENTVALUES Calculates the segments based on marker positions
            %   setSegmentValues( segPos2D )
            %
            %   segPos2D      3xM matrix - center positions of M segments
           
            % Check #1 - are there enough segments and is the position dimension correct?
          

                % init thetaMeas and arcLenMeas!
                obj.thetaMeas(1) = obj.dims.thetaStart;
                
                % Calculate the curvatures and angles using the 2 points method
                for s = 1:obj.dims.S
                    % Calculate properties of the current segment
                    [obj.kMeas(1,s), obj.thetaMeas(s+1)] = Arm2D.singSegIK(...
                        obj.segPos2D(1:2,s),obj.thetaMeas(s), obj.segPos2D(1:2,s+1));
                    obj.arcLenMeas(1,s) = wrapToPi(...
                        obj.thetaMeas(s+1)-obj.thetaMeas(s)) / obj.kMeas(s);
                    
                    % Check #2 - is the calculated length more than 20% different than the
                    % expected length?
                    l_lengthDiff = (obj.arcLenMeas(1,s)-obj.dims.lengths(1,s))/obj.dims.lengths(1,s);
                    if (abs(l_lengthDiff) > 0.20)
                        error('bad arclength for Segment %i: lengt: %f',s,l_lengthDiff);
                    end
                end
                
                %obj.setMeasuredCurvatures(l_kMeas);
                %obj.setMeasuredLengths(l_arcLenMeas);
                % HERE ADD THE GRIPPER ANALYSIS FOR s =
                % (obj.dims.S+1):(obj.dims.S+1+obj.gripper2D.dims.S)
                % FOR NOW HARDCODED /todo
                obj.gripper2D.setMeasuredCurvatures(0.01); 
                obj.gripper2D.setMeasuredLengths(0.113); %arc length in meters
            
            
        end
        
        %Forward kinematic transformation of the 2D arm
        function [x, y, theta] = recursiveForwardKinematics(obj, k, i, s)
            
            % gripper_on - boolean if gripper2D attached
            % k - either measured or target curvature vector
            % i - is the segment of interest
            % s - is the length of interest along segment i
            
            l_N = obj.dims.S + 1; % number of arm links plus the gripper
            l_arcLen = [obj.arcLenMeas, obj.gripper2D.arcLenMeas];
            l_valSize = length(k);
            
            if( i > l_valSize )
                error('The size of k does not match the segment of interest.');
            end
            
            l_thetaStart = obj.dims.thetaStart;
            
            theta_init = zeros(1,l_N);
            x_init = zeros(1,l_N);
            y_init = zeros(1,l_N);
            
            if( i == 1)
                theta_init(i) = l_thetaStart;
                x_init(i) = 0.0;
                y_init(i) = 0.0;
            else
                [x_init(i), y_init(i), theta_init(i)] = obj.recursiveForwardKinematics(k, i-1, l_arcLen(i-1) );
            end
            
            theta = theta_init(i) + k(i)*s;
            x = x_init(i) + sin(theta)/k(i) - sin(theta_init(i))/k(i);
            y = y_init(i) - cos(theta)/k(i) + cos(theta_init(i))/k(i);
            
        end
        %Inverse kinematic transform of the 2D arm
        function [kTarget] = inverseKinematics(obj, xTarget, yTarget, thetaTarget, kGuess)
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = obj.dims.kMin;
            ub = obj.dims.kMax;
            
            l_optTime = tic;
            options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3, 'TolX', 1e-6,'GradObj','on', 'GradConstr', 'off');
            kTarget = fmincon(@cost,kGuess,A,b,Aeq,beq,lb,ub,@noncon,options);
            toc(l_optTime)
            
            function [c,ceq] = noncon(k)
                c = [];                  % nonlinear inequality constraints
                ceq = zeros(1,3);        % nonlinear equalitity constraints
                l_i = obj.dims.S;
                l_s = obj.arcLenMeas(l_i);
                
                [xMeasured, yMeasured, thetaMeasured] = obj.recursiveForwardKinematics(k, l_i, l_s);
                ceq(1) = xMeasured - xTarget;
                ceq(2) = yMeasured - yTarget;
                ceq(3) = thetaMeasured - thetaTarget;
            end
            
            function [E_tot, g] = cost(k)
                E_tot = sum(k.^2);
                g = 2.*k;
            end
        end
        %Plot the target state of the 2D
        function h = plotArmTargetToHandle(obj, k)
            
            
            l_N = obj.dims.S + 1;
            l_arcLen = [obj.arcLenMeas, obj.gripper2D.arcLenMeas];
            l_k = [k, obj.gripper2D.kMeas];
            
            M = 10;
            total = 1;
            x = zeros(1, l_N*M);
            y = zeros(1, l_N*M);
            theta = zeros(1, l_N*M);
            
            for i=1:l_N
                for j=1:M
                    [x(total), y(total), theta(total)] = obj.recursiveForwardKinematics( l_k, i, l_arcLen(i)*(j/M));
                    total = total + 1;
                end
            end
            
            hold on
            axis([-0.30 0.30 -0.10 0.50])
            axis square
            
            h = plot(x(1:end-M),y(1:end-M), 'r', x(end-M:end),y(end-M:end), 'k', 'LineWidth', 2);

            drawnow;
            
        end
        %Plot the measured state of the 2D
        function h = plotArmMeasToHandle(obj, k)
            
            
            l_N = obj.dims.S + 1;
            l_arcLen = [obj.arcLenMeas, obj.gripper2D.arcLenMeas];
            l_k = [k, obj.gripper2D.kMeas];
            
            M = 10;
            total = 1;
            x = zeros(1, l_N*M);
            y = zeros(1, l_N*M);
            theta = zeros(1, l_N*M);
            
            for i=1:l_N
                for j=1:M
                    [x(total), y(total), theta(total)] = obj.recursiveForwardKinematics( l_k, i, l_arcLen(i)*(j/M));
                    total = total + 1;
                end
            end
            
            hold on
            axis([-0.30 0.30 -0.10 0.50])
            axis square
            
            h = plot(x(1:end-10),y(1:end-10), 'b', x(end-10:end),y(end-10:end), 'k', 'LineWidth', 2);

            drawnow;
            
        end
    end
    methods(Static)
        %Single Segment Inverse kinematics
        function [ k, erot ] = singSegIK(spos, srot, epos )
            %SINGSEGIK Calculates the curvature of a segment between spos and epos
            %
            %   INPUT:
            %   spos    2x1 vector - start position
            %   srot        scalar - start rotation
            %   epos    2x1 vector - end position
            %
            %   OUTPUT:
            %   k           scalar - signed curvature between spos and epos
            %   erot        scalar - end rotation
            %
            %   The method involves constructing two lines which are known to be
            %   perpendicular to the arc:
            %
            %       u - which is perpendicular to the arc at point spos (due to known
            %           srot)
            %       v - which is perpendicular to the arc at the point exactly between
            %           spos and epos (the perpendicular bisector of the segment
            %           between them)
            %
            %   Their intersection gives as the position of the center of curvature of
            %   the arc, from which the radius and curvature k = r^-1 can be
            %   calculated. The curvature is signed according to "right hand screw
            %   rule".
            
            % Parametric equation of the first line
            u = [spos [cos(srot + pi/2); sin(srot + pi/2)]];
            
            % Parametric equation of the second line
            v = [(spos + epos) ./ 2 [0 -1; 1 0] * (epos - spos)];
            
            % Value of the second line's parameter at the intersection of the two lines
            lambda_v = ( u(2,2) * (v(1,1) - u(1,1)) - u(1,2) * (v(2,1) - u(2,1)) ) /...
                ( u(1,2) * v(2,2) - u(2,2) * v(1,2) );
            
            if lambda_v == Inf
                % Center of curvature at an infinite distance, zero curvature
                k = 0;
                
                % Out vector
                out_vector = epos - spos;
            else
                % Intersection coordinates
                o = v(:,1) + lambda_v .* v(:,2);
                
                % Cross product of (a-o) and (b-o) in the third dimension
                e = spos-o;
                f = epos-o;
                dir = e(1)*f(2) - e(2)*f(1);
                
                % Signed curvature
                k = sign(dir) * sum((epos - o).^2)^-0.5;
                
                % Cross product of a-c and the direction 'vector'
                out_vector = dir * [-f(2); f(1)];
            end
            
            % End rotation
            erot = atan2(out_vector(2), out_vector(1));
            
            %      if(erot <= 0.0)
            %         erot = 2*pi + erot;
            %      end
            
        end
    end
    
end

