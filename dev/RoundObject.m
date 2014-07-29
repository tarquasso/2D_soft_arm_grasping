classdef RoundObject < handle
    %ROUNDOBJECT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fun = 1;
        prop = 2;
    end
    
    methods
        function obj = RoundObject()
            obj.prop = 2;
        end
        function set.fun(obj,value)
            if ~(value > 0)
                error('Property value must be positive')
            else
                obj.fun = value+obj.prop;
            end
        end
        function delete(obj)
        end
    end
    
end

