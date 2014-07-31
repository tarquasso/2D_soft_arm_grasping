classdef PlannerGrasp < handle
    %PLANNERGRASP Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arm2D;
        grasper2D;
        roundObject;
        
    end
    
    methods
        function obj = PlannerGrasp(arm2DHand,grasper2DHand,roundObjectHand)
           obj.arm2D =  arm2DHand;
           obj.grasper2D = grasper2DHand;
           obj.roundObject = roundObjectHand;
        end
        function delete(obj)
        end
        function plan(obj)
        end
    end
end

