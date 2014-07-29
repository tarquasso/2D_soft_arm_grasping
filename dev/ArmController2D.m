classdef ArmController2D < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arm2D;
        gripper2D;
        roundObject;
        sensor;
        plannerGrasp;
    end
    
    methods
        % Constructor
        function obj = ArmController2D()
            obj.arm2D = Arm2D();
            obj.gripper2D = Gripper2D();
            obj.roundObject = RoundObject();
            obj.sensor = Sensor(obj.arm2D,obj.roundObject);
            obj.plannerGrasp = PlannerGrasp(obj.arm2D,obj.roundObject,obj.sensor);
        end
        function start(obj)
            obj.sensor.initialize();
        end
        % Destructor
        function delete(obj)
            obj.arm2D.delete();
            obj.gripper2D.delete();
            obj.roundObject.delete();
            obj.sensor.delete();
            obj.plannerGrasp.delete();
        end
    end
    
end

