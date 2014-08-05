classdef Gripper2D <handle

    properties
       dims                     % gripper dimensions 
    end
    
    properties(SetAccess=private,GetAccess=public)
        L;
        kMeasured;
        kTarget;
    end
    
    methods
        function obj = Gripper2D()
            obj.dims = struct();
            obj.dims.S = 1;      % is the total number of arm segments
            obj.dims.kMin = -60;     % minimum allowable curvature
            obj.dims.kMax = 60;      % maximum allowable curvature
        end
        
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
                else
                    error('An element of k exceeds allowable gripper limit')
                end
                
            else
                error('The size of k does not match the gripper.')
            end
        end
        
        function setMeasuredCurvatures(obj, val)
            l_valSize = size(val);
            if( l_valSize(1) == obj.dims.S && l_valSize(2) == 1)
                obj.kMeasured = val;
            else
                error('The size of measured k does not match the gripper.');
            end
        end
        
        function setMeasuredLengths(obj, val)
            l_valSize = size(val);
            if( l_valSize(1) == obj.dims.S && l_valSize(2) == 1)
                obj.L = val;
            else
                error('The size of measured L does not match the gripper.');
            end
        end
        
        function delete(obj)
        end
        
    end
    
end

