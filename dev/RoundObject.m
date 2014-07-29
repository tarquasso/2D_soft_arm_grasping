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
        function delete(obj)
            
        end
    end
    
end

