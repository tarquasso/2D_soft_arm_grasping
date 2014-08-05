classdef PlannerGrasp < handle
    
    properties
        arm2D;
        roundObject;
    end
    
    properties(SetAccess=private,GetAccess=public)
        state;
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
        
        alignmentTipTransitTime;
        advancementTipTransitTime;
        graspTime;
        planTime;
        
    end
    
    methods
        %Constructor
        function obj = PlannerGrasp(arm2DHand,roundObjectHand)
            obj.arm2D =  arm2DHand;
            obj.roundObject = roundObjectHand;
            obj.state = 1;
            obj.previousObjPosition = [0; 0];
            obj.changeInObjPositionThreshold = 0.005; % half of a centimeter
            obj.timeAtObjPosition = 0.0;
            obj.timeAtObjPositionThreshold = 4.0;
            
            obj.initialTipPoseComputed = 0;
            obj.initialTipPose = zeros(3,1);
            
            obj.alignmentTipPoseComputed = 0;
            obj.alignmentTipPose = zeros(3,1);
            obj.advancedTipPose = zeros(3,1);
            obj.graspCurvature = obj.arm2D.gripper2D.dims.kMax;
            
            obj.alignmentTipTransitTime = 5.0;
            obj.advancementTipTransitTime = 5.0;
            obj.graspTime = 3.0;
            
        end
        %Destructor
        function delete(obj)
        end
        %Plan dispatcher
        function plan(obj)
            
            switch obj.state
                case 1 % object is NOT placed
                    obj.checkObjPlacement();
                case 2 % tip is NOT aligned
                    obj.alignArmTip();
                case 3 % tip is NOT advanced
                    obj.advanceArmTip();
                case 4 % object is NOT grasped
                    obj.graspObject();
                otherwise
                    display('plan executed')
            end
            
        end
        %Determine whether or not and object has been placed
        function checkObjPlacement(obj)
            
            l_currentObjPosition = [obj.roundObject.x; obj.roundObject.y];
            
            if( norm(obj.previousObjPosition,2) >= 0) % prev not at default location
                
                l_changeInObjPosition = norm( l_currentObjPosition - obj.previousObjPosition, 2);
                
                if( l_changeInObjPosition <= obj.changeInObjPositionThreshold )
                    if( obj.timeAtObjPosition >= obj.timeAtObjPositionThreshold ) % has it been engough time?
                        display('Object is placed')
                        obj.state = 2;
                    else
                        display('Waiting for object to settle')
                    end
                    obj.timeAtObjPosition = obj.timeAtObjPosition + toc(obj.planTime);
                else
                    obj.timeAtObjPosition = 0;
                    display('Object location is changing')
                end
                
                obj.planTime = tic; % get current time
                
            end
            
            obj.previousObjPosition = l_currentObjPosition; %set to current position
            
        end
        %Determine and drive arm tip to an alignment pose
        function alignArmTip(obj)
            
            if( obj.initialTipPoseComputed == 0 ) % compute initial tip pose
                l_k = obj.arm2D.kMeasured;
                l_L = obj.arm2D.L;
                l_i = obj.arm2D.dims.S;
                
                [obj.initialTipPose(1), obj.initialTipPose(2), obj.initialTipPose(3)] = obj.arm2D.recursiveForwardKinematics(l_k, l_i, l_L(l_i));
                plot(obj.initialTipPose(1), obj.initialTipPose(2), 'ok', 'MarkerSize', 10);
                obj.initialTipPoseComputed = 1;
            end
            
            if( obj.alignmentTipPoseComputed == 0 ) % compute alignement tip pose
                l_D = sqrt(obj.roundObject.x^2 + obj.roundObject.y^2);        % distance from ground to object center
                l_angle = atan2(obj.roundObject.y, obj.roundObject.x)-pi/2;   % angle between ground and object center
                l_perpOffset = 2*obj.roundObject.r;                           % perpendicular offset from object
                l_parallelOffset = obj.arm2D.gripper2D.L;                     % parallel offset from object
                l_parallelAdvment = 0;                                        % parallel advancement amount
                
                obj.alignmentTipPose(1) = l_perpOffset*cos(l_angle) - (l_D - l_parallelOffset - obj.roundObject.r)*sin(l_angle) - l_parallelAdvment*sin(l_angle);
                obj.alignmentTipPose(2) = (l_D - l_parallelOffset - obj.roundObject.r)*cos(l_angle) + l_parallelAdvment*cos(l_angle) + l_perpOffset*sin(l_angle);
                obj.alignmentTipPose(3) = l_angle + obj.arm2D.dims.theta0;
                plot(obj.alignmentTipPose(1), obj.alignmentTipPose(2), 'ok', 'MarkerSize', 10);
                
                l_parallelAdvment = 0.75*obj.arm2D.gripper2D.L;               % parallel advancement amount
                obj.advancedTipPose(1) = l_perpOffset*cos(l_angle) - (l_D - l_parallelOffset - obj.roundObject.r)*sin(l_angle) - l_parallelAdvment*sin(l_angle);
                obj.advancedTipPose(2) = (l_D - l_parallelOffset - obj.roundObject.r)*cos(l_angle) + l_parallelAdvment*cos(l_angle) + l_perpOffset*sin(l_angle);
                obj.advancedTipPose(3) = l_angle + obj.arm2D.dims.theta0;
                plot(obj.advancedTipPose(1), obj.advancedTipPose(2), 'ok', 'MarkerSize', 10);
                obj.alignmentTipPoseComputed = 1;
            end
            
            %%%%%%%%%%%%%%%%% is manipulator tip aligned? %%%%%%%%%%%%%%%%%
            l_k = obj.arm2D.kMeasured;
            l_L = obj.arm2D.L;
            l_i = obj.arm2D.dims.S;
            [l_measuredX, l_measuredY, l_measuredTheta] = obj.arm2D.recursiveForwardKinematics(l_k, l_i, l_L(l_i));
            
            %%%%%%%%%%%%%%%%% if the tip pose is aligned %%%%%%%%%%%%%%%%%%
            if( norm( [l_measuredX; l_measuredY] - [obj.alignmentTipPose(1); obj.alignmentTipPose(2)],2 ) <= 0.01 ...
                    && norm( l_measuredTheta - obj.alignmentTipPose(3), 2) <= 0.1 )
                obj.state = 3; %then move to next planning state
                display('Arm is aligned');
                obj.planTime = tic; % get current time
                
                %%%%%%%%%%%%%%%%% drive arm to aligned pose %%%%%%%%%%%%
            else
                if( toc(obj.planTime) <= obj.alignmentTipTransitTime )
                    
                    l_xTarget = obj.linInterpolate(toc(obj.planTime), obj.alignmentTipTransitTime, obj.initialTipPose(1), obj.alignmentTipPose(1));
                    l_xTarget = obj.linInterpolate(toc(obj.planTime), obj.alignmentTipTransitTime, obj.initialTipPose(1), l_xTarget );
                    l_yTarget = obj.linInterpolate(toc(obj.planTime), obj.alignmentTipTransitTime, obj.initialTipPose(2), obj.alignmentTipPose(2));
                    l_thetaTarget = obj.linInterpolate(toc(obj.planTime), obj.alignmentTipTransitTime, obj.initialTipPose(3), obj.alignmentTipPose(3));
                    plot(l_xTarget, l_yTarget, 'og', 'MarkerSize', 10);
                    
                    l_kGuess = obj.arm2D.kTarget;
                    [l_kTarget] = obj.arm2D.inverseKinematics(l_xTarget, l_yTarget, l_thetaTarget, l_kGuess);
                    obj.arm2D.setTargetCurvatures(l_kTarget);
                else
                    l_kGuess = obj.arm2D.kTarget;
                    [l_kTarget] = obj.arm2D.inverseKinematics(obj.alignmentTipPose(1), obj.alignmentTipPose(2), obj.alignmentTipPose(3), l_kGuess);
                    obj.arm2D.setTargetCurvatures(l_kTarget);
                end
            end
        end
        %Determine and drive arm tip to an advanced pose alongside object
        function advanceArmTip(obj)
            %%%%%%%%%%%%%%%%% is manipulator tip advanced? %%%%%%%%%%%%%%%%%
            l_k = obj.arm2D.kMeasured;
            l_L = obj.arm2D.L;
            l_i = obj.arm2D.dims.S;
            [l_measuredX, l_measuredY, l_measuredTheta] = obj.arm2D.recursiveForwardKinematics(l_k, l_i, l_L(l_i));
            
            %%%%%%%%%%%%%%%%%%%%% if the tip is advanced %%%%%%%%%%%%%%%%%%
            if( norm( [l_measuredX; l_measuredY] - [obj.advancedTipPose(1); obj.advancedTipPose(2)],2 ) <= 0.01 ...
                    && norm( l_measuredTheta - obj.advancedTipPose(3), 2) <= 0.1 )
                obj.state = 4; %then move to next planning state
                display('Arm is advanced');
                obj.planTime = tic; % get current time
                
                %%%%%%%%%%%%%%%%% otherwise advance the arm %%%%%%%%%%%%%%%
            else
                %%%%%%%%%%%%%%%%% drive arm to alongside object %%%%%%%%%%%%
                if( toc(obj.planTime) <= obj.advancementTipTransitTime )
                    l_xTarget = obj.linInterpolate(toc(obj.planTime), obj.advancementTipTransitTime, obj.alignmentTipPose(1), obj.advancedTipPose(1));
                    l_yTarget = obj.linInterpolate(toc(obj.planTime), obj.advancementTipTransitTime, obj.alignmentTipPose(2), obj.advancedTipPose(2));
                    l_thetaTarget = obj.linInterpolate(toc(obj.planTime), obj.advancementTipTransitTime, obj.alignmentTipPose(3), obj.advancedTipPose(3));
                    plot(l_xTarget, l_yTarget, 'om', 'MarkerSize', 10);
                    
                    l_kGuess = obj.arm2D.kTarget;
                    [l_kTarget] = obj.arm2D.inverseKinematics(l_xTarget, l_yTarget, l_thetaTarget, l_kGuess);
                    obj.arm2D.setTargetCurvatures(l_kTarget);
                else
                    l_kGuess = obj.arm2D.kTarget;
                    [l_kTarget] = obj.arm2D.inverseKinematics(obj.advancedTipPose(1), obj.advancedTipPose(2), obj.advancedTipPose(3), l_kGuess);
                    obj.arm2D.setTargetCurvatures(l_kTarget);
                end
            end
        end
        %Actuate the gripper to grasp object
        function graspObject(obj)
            %%%%%%%%%%%%%%%% is gripper at grasp curvature? %%%%%%%%%%%%%%%
            l_k = obj.arm2D.gripper2D.kMeasured;
            
            %%%%%%%%%%%%%%%%% if it is at grasp curvature %%%%%%%%%%%%%%%%%
            if( norm( l_k - obj.graspCurvature, 2 ) <= 1 )
                obj.state = 5; %then move to next planning state
                display('Gripper has grasped');
                obj.planTime = tic; % get current time
                
                %%%%%%%%%%%% otherwise advance gripper curvature %%%%%%%%%%%%%%
            else
                if( toc(obj.planTime) <= obj.graspTime )
                    l_kInit = 0; % <---- Unactauted gripper curvature
                    l_kTarget = obj.linInterpolate(toc(obj.planTime), obj.graspTime, l_kInit, obj.graspCurvature);
                    obj.arm2D.gripper2D.setTargetCurvatures(l_kTarget);
                else
                    obj.arm2D.gripper2D.setTargetCurvatures( obj.graspCurvature );
                end
            end
        end
        %An interporlation function
        function valBetween = linInterpolate(obj, tCurrent, tMax, valInitial, valFinal)
            valBetween = valInitial + (valFinal - valInitial)*(tCurrent)/(tMax);
        end
    end
end

