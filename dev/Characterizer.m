classdef Characterizer < handle
    
    properties
        arm2D;
        framePeriod;
        shapeHistory;
    end
    
    properties(SetAccess=private,GetAccess=public)
        plannerFree;
        state;
        objInRightPosition;
        previousObjPosition;
        changeInObjPositionThreshold;
        timeAtObjPosition;
        timeAtObjPositionThreshold;
        
        initialTipPoseComputed;
        initialTipPose;
        alignmentTipPoseComputed;
        alignmentTipPose;
        advancedTipPose;
        graspCurvature;
        
        %alignmentTipTransitTime;
        %advancementTipTransitTime;
        alignmentPathProfileComputed;
        alignmentPathProfile;
        alignmentTipTransitDistance;
        
        advancementPathProfileComputed;
        advancementPathProfile;
        advancementTipTransitDistance;
        
        graspTime;
        planTime;
        startedToSettle;
        startedToGrasp;
        startedToRelease;
        
        vMax;
        aMax;
        
        %ArcSpace Planner
        transitDistInc;
        posMoveEpsilon;
        posMoveEpsilonFinal;
        rotMovEpsilon;
        rotMovEpsilonFinal;
        arcSpacePlanDone;
        nMov;
        moveTo;
        kOptimal;
        gammaOptimal;
        tipOptimal;
        trajGenerated;
        kInit;
        kGoal;
        curvatureProfiles;
        trajectoryEndTime;
        waitTimeForSettle;
        kOff1;
        kOff5;
        kOff6;
        waitTimeForGrasp;
        binTrajGenerated;
        xBin;
        yBin;
        stateTimeInit;
        allRadii;
        %only for plotting
        connectLine;
        
    end
    
    methods
        %Constructor
        function obj = Characterizer(arm2DHand)
            obj.arm2D = arm2DHand;
            S = obj.arm2D.dims.S;
            N = 60*100; % max data points to log
            obj.shapeHistory = ShapeHistory(S, N);
            
        end
        %Destructor
        function delete(obj)
            % Save the shape history
            filename = sprintf('data\\%s.mat', datestr(now));
            filename = strrep(filename,':','_');
            History = obj.shapeHistory;
            save(filename, 'History');
            
            %delete shape history
            delete(obj.shapeHistory);
        end
        %step dispatcher
        function result = step(obj)
            result = 0;
            obj.logData();          
        end
        
        function logData(obj)
            if( isempty( obj.stateTimeInit ) )
                obj.stateTimeInit = tic;
            end
            l_N = obj.arm2D.dims.S;
            l_time = toc(obj.stateTimeInit);

            obj.shapeHistory.add(l_time, obj.arm2D.kMeas, obj.arm2D.arcLenMeas, ...
                NaN(1, l_N), [obj.arm2D.segPos2D(1,l_N+1), obj.arm2D.segPos2D(2,l_N+1), obj.arm2D.thetaMeas(1, l_N)], ...
                NaN(1,2), NaN(1,1));
        end
             
    end
end

