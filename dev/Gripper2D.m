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
            obj.dims.kThreshold = 4;
            obj.calibrated = false;
            %Initialize with some values  
            obj.setMeasuredLengths(4.0*0.0254);
            obj.setMeasuredCurvatures(0.0);
            obj.setTargetCurvatures(0.0);
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
                    if( (val(i) < obj.dims.kMin) || (val(i) > obj.dims.kMax) )
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
            [l_kMeas, obj.thetaMeas(1,2)] = Arm2D.singSegIK(...
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
    
end

