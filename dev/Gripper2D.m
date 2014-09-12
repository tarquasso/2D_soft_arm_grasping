classdef Gripper2D <handle
    
    properties
        dims                     % gripper dimensions
        segPos2D
    end
    
    properties(SetAccess=private,GetAccess=public)
        
        arcLenMeas;
        kMeas;
        kTarget;
    end
    
    methods
        %Constructor
        function obj = Gripper2D()
            %Define Gripper dimensions
            obj.dims = struct();
            obj.dims.S = 1;      % is the total number of arm segments
            obj.dims.kMin = 0;     % minimum allowable curvature
            obj.dims.kMax = 40;      % maximum allowable curvature
            obj.dims.offCenter = 0.007; %gripper offset distance in meters
            
            %Initialize with some values  
            obj.setMeasuredLengths(4.0*0.0254);
            obj.setMeasuredCurvatures(1.0);
            obj.setTargetCurvatures(1.0);            
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
                    error('An element of k exceeds allowable gripper limit')
                end
                
            else
                error('The size of k does not match the gripper.')
            end
        end
        %Set measured curvature of the gripper
        function setMeasuredCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.kMeas = val;
            else
                error('The size of measured k does not match the gripper.');
            end
        end
        %Set measured length of the gripper
        function setMeasuredLengths(obj, val)
            l_valSize = size(val);
            if( l_valSize(2) == obj.dims.S && l_valSize(1) == 1)
                obj.arcLenMeas = val;
            else
                error('The size of arcLenMeas does not match the gripper.');
            end
        end
    end
    
end

