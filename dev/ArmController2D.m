classdef ArmController2D < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties(Access=public)
        arm2D;
        baseBoard;
        roundObject;
        sensor;
        trajGen;
        plannerGrasp;
        simulationTime;
        shapeHistory;
        expType;
        
        armPlotHandleTarget;
        armPlotHandleMeas;
        objectPlotHandle;
        
    end
    
    methods(Access = public)
        % Constructor
        function obj = ArmController2D(expType)
             if nargin < 1
                obj.expType = ExpTypes.PhysicalExperiment;
            else
                if(isa(expType,'ExpTypes'))
                    obj.expType = expType;
                else
                    error('wrong type');
                end
            end
            obj.expType = expType;
            obj.arm2D = Arm2D( obj.expType );
            %obj.baseBoard = BaseBoard();
            obj.roundObject = RoundObject();
            obj.sensor = Sensor(obj.arm2D,obj.roundObject,obj);
            obj.trajGen = TrajGen();
            obj.plannerGrasp = PlannerGrasp(PlannerTypes.ArcSpacePlanner,...
                obj.arm2D,obj.roundObject, obj.trajGen, 1/obj.sensor.frameRate ); % last parameter is frame period
            obj.simulationTime = 45;
            obj.shapeHistory = ShapeHistory(obj.arm2D.dims.S, ...
                obj.simulationTime * obj.sensor.frameRate);
            
        end
        function start(obj)
            % attach the frame callback to start the sensor
            obj.sensor.start();
        end
        function stop(obj)
            % attach the frame callback to start the sensor
            obj.sensor.stop();
        end
        function sensorMeasurementsDone(obj)
            if(obj.expType == ExpTypes.PhysicalExperiment || obj.expType == ExpTypes.Simulation)
                l_result = obj.plannerGrasp.plan();
                
                if( isempty(obj.armPlotHandleTarget) )
                    obj.armPlotHandleTarget = obj.arm2D.plotArmTargetToHandle(obj.arm2D.kTarget);
                    obj.armPlotHandleMeas = obj.arm2D.plotArmMeasToHandle(obj.arm2D.kMeas);
                    obj.objectPlotHandle = obj.roundObject.plotObjectToHandle();
                else
                    %plots the intermediate trajectory points
                    %                 delete(obj.armPlotHandleTarget);
                    %                 obj.armPlotHandleTarget = obj.arm2D.plotArmTargetToHandle(obj.arm2D.kTarget);
                    
                    delete(obj.armPlotHandleMeas);
                    obj.armPlotHandleMeas = obj.arm2D.plotArmMeasToHandle(obj.arm2D.kMeas);
                    
                    delete(obj.objectPlotHandle);
                    obj.objectPlotHandle = obj.roundObject.plotObjectToHandle();
                end
                
                if(l_result == 1)
                    obj.stop();
                    obj.delete();
                end
            end
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

