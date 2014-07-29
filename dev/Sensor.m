classdef Sensor < handle
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arm2D;
        roundObject;
    end
    
    methods
        function obj = Sensor(arm2DHandle,roundObjectHandle)
            obj.arm2D = arm2DHandle;
            obj.roundObject = roundObjectHandle;
            
        end
        function delete(obj)
        end
        
    end
    
end

