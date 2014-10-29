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
        characterizer;
        
        armPlotHandleTarget;
        armPlotHandleMeas;
        objectPlotHandle;
        
    end
    
    methods(Access = public)
        % Constructor
        function obj = ArmController2D(expType, armDof, gripperExists)
                
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
            obj.arm2D = Arm2D( obj.expType , armDof, gripperExists);
            
            if(obj.expType == ExpTypes.Characterization)
                %no round object
                obj.sensor = Sensor(obj,obj.arm2D);
            else
                obj.roundObject = RoundObject();
                obj.sensor = Sensor(obj,obj.arm2D,obj.roundObject);
            end
            
            if(obj.expType == ExpTypes.Characterization)
                obj.characterizer = Characterizer(obj.arm2D);
            else
                obj.trajGen = TrajGen();
                obj.plannerGrasp = PlannerGrasp(PlannerTypes.ArcSpacePlanner,...
                    obj.arm2D,obj.roundObject, obj.trajGen, 1/obj.sensor.frameRate ); % last parameter is frame period
            end
%             obj.simulationTime = 45;
%             obj.shapeHistory = ShapeHistory(obj.arm2D.dims.S, ...
%                 obj.simulationTime * obj.sensor.frameRate);
            
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
                if(obj.arm2D.calibrated == true && obj.arm2D.gripper2D.calibrated == true) 
                    l_result = obj.plannerGrasp.plan();
                    obj.plannerGrasp.logData();
                else
                    l_result = 0;
                end
                
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
                
            elseif(obj.expType == ExpTypes.Characterization)
                if(obj.arm2D.calibrated == true)
                    l_result = obj.characterizer.step();
                    %obj.plannerGrasp.logData();
                else
                    l_result = 0;
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
            if(obj.expType ~= ExpTypes.Characterization)
            obj.roundObject.delete();
            end
            obj.sensor.delete();
            if(obj.expType ~= ExpTypes.Characterization)
            obj.plannerGrasp.delete();
            end
%             obj.shapeHistory.delete();
        end
    end
end

