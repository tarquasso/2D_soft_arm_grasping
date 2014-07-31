classdef ArmController2D < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        arm2D;
        baseBoard;
        roundObject;
        sensor;
        plannerGrasp;
        simulationTime;
        shapeHistory;
    end
    
    methods(Access = public)
        % Constructor
        function obj = ArmController2D()
            obj.arm2D = Arm2D();
            obj.baseBoard = BaseBoard();
            obj.roundObject = RoundObject();
            obj.sensor = Sensor(obj.arm2D,obj.roundObject);
            obj.plannerGrasp = PlannerGrasp(obj.arm2D,obj.roundObject,obj.sensor);
            obj.simulationTime = 45;
            obj.shapeHistory = ShapeHistory(obj.arm2D.armDims.S, ...
                obj.simulationTime * obj.sensor.frameRate);

        end
        function start(obj)
            % attach the frame callback to start the sensor
            obj.sensor.start();
        end
        % Destructor
        function delete(obj)
            obj.arm2D.delete();
            obj.roundObject.delete();
            obj.sensor.delete();
            obj.plannerGrasp.delete();
            obj.shapeHistory.delete();
        end
    end  
end

