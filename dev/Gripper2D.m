classdef Gripper2D <handle
    
    properties
        dims                     % gripper dimensions
        segPos2D
    end
    
    properties(SetAccess=private,GetAccess=public)
        
        arcLenMeas;
        thetaMeas;
        kMeas;
        kTarget;
        
        calibrated;
        kMeasInit;
    end
    
    methods
        %Constructor
        function obj = Gripper2D()
            %Define Gripper dimensions
            obj.dims = struct();
            obj.dims.S = 1;      % is the total number of arm segments
            obj.dims.kMin = -10;     % minimum allowable curvature
            obj.dims.kMax = 40;      % maximum allowable curvature
            obj.dims.offCenter = 0.007; %gripper offset distance in meters
            obj.dims.kThreshold = 5;
            obj.dims.kEpsilon = 1;
            obj.calibrated = false;
            %Initialize with some values  
            obj.setMeasuredLengths(4.0*0.0254);
            obj.setMeasuredCurvatures(0.01);
            obj.setTargetCurvatures(0.01);
            obj.kMeasInit = 1.0;
            obj.thetaMeas = zeros(1,2);
        end
        %Destructor
        function delete(obj)
        end
        %Set target curvature of the gripper
        function setTargetCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                
                above_max = 'false';
                
                for i = 1:obj.dims.S
                    if( (val(i) < obj.dims.kMin-obj.dims.kEpsilon) || (val(i) > obj.dims.kMax+obj.dims.kEpsilon) )
                        above_max = 'true';
                    end
                end
                
                if( strcmp(above_max, 'false') )
                    obj.kTarget = val;
                else
                    display('ERROR: [Gripper2D-setTargetCurvatures] An element of k exceeds allowable gripper limit')
                end
                
            else
                display('ERROR: [Gripper2D-setTargetCurvatures] The size of k does not match the gripper.')
            end
        end
        %Set measured curvature of the gripper
        function setMeasuredCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.kMeas = val;
            else
                display('ERROR: [Gripper2D-setMeasuredCurvatures] The size of measured k does not match the gripper.');
            end
        end
        %Set measured length of the gripper
        function setMeasuredLengths(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.arcLenMeas = val;
            else
                display('ERROR: [Gripper2D-setMeasuredLengths]The size of arcLenMeas does not match the gripper.');
            end
        end
        
        function calculateSegmentValues( obj, thetaMeasStart)
            obj.thetaMeas(1,1) = thetaMeasStart;
            [l_kMeas, obj.thetaMeas(1,2)] = gripper2D.singSegIK(...
                obj.segPos2D(1:2,1),obj.thetaMeas(1,1), obj.segPos2D(1:2,2));
             if (l_kMeas < obj.dims.kMin)
             l_kMeas = abs(l_kMeas);
             end
            
            if(obj.calibrated == false)
                if( max(abs(l_kMeas) > obj.dims.kThreshold) == 1 )                    
                    fprintf('gripper not at home!!: '); 
                    l_kMeas
                    fprintf('\n');
                else
                    obj.calibrated = true;
                    obj.kMeasInit = l_kMeas;
                end
            end
            
            %obj.setMeasuredCurvatures(obj.kTarget);
            obj.setMeasuredCurvatures(l_kMeas-obj.kMeasInit);
            
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
            
            if(out_vector(2) < 0.0 && out_vector(1) < 0.0 )
                erot = pi + (pi + erot);
            end
        end
    end
end

