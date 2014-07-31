classdef Arm2D < handle
    %ARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        theta0 = pi/2;           % is the current/measured initial orientation of the first segment
        L = [];                  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        above_max = 'false'      % boolean if any segments are above k limits
        k_measured               % measured arc curvatures
        curvatureController     % curvature controller
        k_target                 % target arc curvatures
    end
    
    properties(Dependent)
        
    end
    
    properties(Constant)
        N = 6;                   % is the total number of arm segments
        k_min = -20;             % minimum allowable curvature
        k_max = 20;              % maximum allowable curvature
    end
    
    methods
        
        function obj = Arm2D()
            obj.curvatureController = CurvatureController;
        end      
        function obj = setTarget(obj, val)
            dims = size(val);
            if( dims(1) == obj.N && dims(2) == 1)
                strcmp(obj.above_max, 'false');
                
                for i = 1:obj.N
                    if( (val(i) < obj.k_min) || (val(i) > obj.k_max) )
                        obj.above_max = 'true';
                    end
                end
                
                if( strcmp(obj.above_max, 'false') )
                    obj.k_target = val;
                    obj.curvatureController.sendCurvatureErrors(...
                        obj.k_target', obj.k_measured);
                else
                    error('An element of k exceeds allowable limit')
                end
                
            else
                error('The size of k does not match the arm.')
            end
        end
        
        function obj = setMeasured(obj, k, L)
            obj.k_measured = k;
            obj.L = L;
        end
        
    end
    
    
end

