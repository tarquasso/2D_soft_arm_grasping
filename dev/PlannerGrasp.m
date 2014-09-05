classdef PlannerGrasp < handle
    
    properties
        arm2D;
        roundObject;
        framePeriod;
        plannerType;
    end
    
    properties(SetAccess=private,GetAccess=public)
        plannerFree;
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
        

        vMax;
        aMax; 
    end
    
    methods
        %Constructor
        function obj = PlannerGrasp(plannerTypeInput,arm2DHand,roundObjectHand, framePeriod)
            obj.arm2D =  arm2DHand;
            obj.roundObject = roundObjectHand;
            obj.framePeriod = framePeriod;
            
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
            
            obj.alignmentPathProfileComputed = 0;
            obj.advancementPathProfileComputed = 0;
            obj.graspTime = 3.0;
            
            obj.vMax = 0.005;
            obj.aMax = 0.001;
            
            obj.plannerFree = 'true';
            obj.plannerType = plannerTypeInput;
        end
        %Destructor
        function delete(obj)
        end
        %Plan dispatcher
        function plan(obj)
            
            if( obj.plannerFree)
                obj.plannerFree = 'false';
                
                switch obj.state
                    case 1 % object is NOT placed
                        obj.checkObjPlacement();
                    case 2 % tip is NOT aligned
                        switch obj.plannerType
                            case PlannerTypes.CartesianPlanner
                                obj.alignArmTip();
                            case PlannerTypes.ArcSpacePlanner
                                obj.arcSpacePlan();
                        end
                    case 3 
                        obj.advanceArmTip();
                    case 4 % object is NOT grasped
                        obj.graspObject();
                    otherwise
                        display('plan executed')
                end
                obj.plannerFree = 'true';
            else
                obj.arm2D.actuate();
            end
            
        end
        
        function arcSpacePlan(obj)
           
            %% TODO
            % Determine concentric circle radii R = [R_1, R_2, R_3, R_4]
            % determine the number of radii (I), control loop iteration =
            % did we get to the set point
            
            % Find local optimal curvatures that satisfy constraints for
            % cylce i
            
            i = 1;
           
            %% COMPLETE minus debugging
            A = []; % A, b, Aeq, and beq all used for linear constraints, intentionally empty here
            b = [];
            Aeq = [];
            beq = [];
            
            % lower and upper bound on decision parameters [theta, curvatures]
            % theta = end effector angle - pi/2
            lb = [0, obj.arm2D.dims.kMin];
            ub = [2*pi, obj.arm2D.dims.kMax];
            
            % we assume the arm settles at the optimal k vector before the
            % next cylce of optimization is called
            % kOptimal = I x dims.S
            % gammaOptimal = I x 1
            
            if( isempty(kOptimal) )
                kGuess = obj.arm2D.kMeas;
                gammaGuess = obj.arm2D.thetaMeas(end);
            else
                kGuess = kOptimal(i-1, :);
                gammaGuess = gammaOptimal(i-1, 1);
            end
            
            guessParameters = [gammaGuess, kGuess];
 
            options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3, 'TolX', 1e-6,'GradObj','on', 'GradConstr', 'off');
            optimalParameters = fmincon(@cost,guessParameters,A,b,Aeq,beq,lb,ub,@noncon,options);
            
            kOptimal(i,:) = optimalParameters(2:end);
            gammaOptimal(i,:) = optimalParameters(1);
            
            function [c,ceq] = noncon(parametersCurrent)
                gamma = parametersCurrent(1);
                k = parametersCurrent(2:end);
                
                xTarget = obj.roundObject.x + R*cos(gamma);
                yTarget = obj.roundObject.y + R*sin(gamma);
                thetaTarget = pi/2 + gamma;
                
                ceq = zeros(1,3);        % nonlinear equalitity constraints
                
                l_i = obj.arm2D.dims.S;
                l_s = obj.arm2D.arcLenMeas(l_i);
                
                % End effector nonlinear equality constraints <- enforce
                % tip position 
                [xTipCurrent, yTipCurrent, thetaTipCurrent] = obj.arm2D.recursiveForwardKinematics(k, l_i, l_s);
                ceq(1) = xTipCurrent - xTarget;
                ceq(2) = yTipCurrent - yTarget;
                ceq(3) = thetaTipCurrent - thetaTarget;
                
                % TODO: rewrite recursive FK to return intermediate
                % endpoints so that we dont have to call it twice 
                
                %  nonlinear inequality constraints on segment N-1 <- ensure wrist object
                %  avoidance
                [xWristCurrent, yWristCurrent, thetaWristCurrent] = obj.arm2D.recursiveForwardKinematics(k, l_i, l_s-1);
                objectCenter = [obj.roundObject.x; obj.roundObject.y];
                c = obj.roundObject.r - norm([xWristCurrent; yWristCurrent] - objectCenter, 2); 
            end
            
            function [E_tot, g] = cost(parametersCurrent)
                k = parametersCurrent(2:end);
                E_tot = sum(k.^2);
                g = 2.*k;
            end
            
            
        end
        
        %Determine whether or not and object has been placed
        function checkObjPlacement(obj)
            
            % Target = Measured for arm and gripper
            obj.arm2D.setTargetCurvatures(obj.arm2D.kMeas);
            obj.arm2D.gripper2D.setTargetCurvatures(obj.arm2D.gripper2D.kMeas);
            obj.arm2D.actuate(); %set target to be equal to be the measured value
            
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
                l_k = obj.arm2D.kMeas;
                l_L = obj.arm2D.arcLenMeas;
                l_i = obj.arm2D.dims.S;
                
                [obj.initialTipPose(1), obj.initialTipPose(2), obj.initialTipPose(3)] = obj.arm2D.recursiveForwardKinematics(l_k, l_i, l_L(l_i));
                plot(obj.initialTipPose(1), obj.initialTipPose(2), '.k', 'MarkerSize', 30);
                obj.initialTipPoseComputed = 1;
            end
            
            if( obj.alignmentTipPoseComputed == 0 ) % compute alignement tip pose
                l_D = sqrt(obj.roundObject.x^2 + obj.roundObject.y^2);        % distance from ground to object center
                l_angle = atan2(obj.roundObject.y, obj.roundObject.x)-pi/2;   % angle between ground and object center
                l_perpOffset = 2*obj.roundObject.r;                           % perpendicular offset from object
                l_parallelOffset = 0.5*obj.arm2D.gripper2D.arcLenMeas;                     % parallel offset from object
                l_parallelAdvment = 0;                                        % parallel advancement amount
                
                obj.alignmentTipPose(1) = l_perpOffset*cos(l_angle) - (l_D - l_parallelOffset - obj.roundObject.r)*sin(l_angle) - l_parallelAdvment*sin(l_angle);
                obj.alignmentTipPose(2) = (l_D - l_parallelOffset - obj.roundObject.r)*cos(l_angle) + l_parallelAdvment*cos(l_angle) + l_perpOffset*sin(l_angle);
                obj.alignmentTipPose(3) = l_angle + obj.arm2D.dims.thetaStart;
                plot(obj.alignmentTipPose(1), obj.alignmentTipPose(2), '.k', 'MarkerSize', 30);
                
                l_parallelAdvment = 0.25*obj.arm2D.gripper2D.arcLenMeas;               % parallel advancement amount
                obj.advancedTipPose(1) = l_perpOffset*cos(l_angle) - (l_D - l_parallelOffset - obj.roundObject.r)*sin(l_angle) - l_parallelAdvment*sin(l_angle);
                obj.advancedTipPose(2) = (l_D - l_parallelOffset - obj.roundObject.r)*cos(l_angle) + l_parallelAdvment*cos(l_angle) + l_perpOffset*sin(l_angle);
                obj.advancedTipPose(3) = l_angle + obj.arm2D.dims.thetaStart;
                plot(obj.advancedTipPose(1), obj.advancedTipPose(2), '.k', 'MarkerSize', 30);
                obj.alignmentTipPoseComputed = 1;
            end
            
            if( obj.alignmentPathProfileComputed == 0 ) % compute alignement path profile
                d = norm([obj.advancedTipPose(1:2)-obj.initialTipPose(1:2)], 2);
                obj.alignmentPathProfile = obj.generateVelocityProfile( d );
                obj.alignmentTipTransitDistance = d;
                obj.alignmentPathProfileComputed = 1;
            end
            
            %%%%%%%%%%%%%%%%% determine end effector pose %%%%%%%%%%%%%%%%%
            l_k = obj.arm2D.kMeas;
            l_L = obj.arm2D.arcLenMeas;
            l_i = obj.arm2D.dims.S;
            [l_measuredX, l_measuredY, l_measuredTheta] = obj.arm2D.recursiveForwardKinematics(l_k, l_i, l_L(l_i));
            
            %%%%%%%%%%%%%%%%% if the tip pose is aligned %%%%%%%%%%%%%%%%%%
            if( norm( [l_measuredX; l_measuredY] - [obj.alignmentTipPose(1); obj.alignmentTipPose(2)],2 ) <= 0.02 ...
                    && norm( l_measuredTheta - obj.alignmentTipPose(3), 2) <= 7.5*(180/3.14159) )
                obj.state = 3; %then move to next planning state
                display('Arm is aligned');
                obj.planTime = tic; % get current time
                
            %%%%%%%%%%%%%%%%%%%% drive arm to aligned pose %%%%%%%%%%%%%%%%%
            else
                l_tCurrent = toc(obj.planTime);
                l_positionDelta = obj.getPositionDelta(obj.alignmentPathProfile, l_tCurrent);
                
                l_xTarget = obj.linInterpolate( l_positionDelta, obj.alignmentTipTransitDistance, obj.initialTipPose(1), obj.alignmentTipPose(1));
                l_yTarget = obj.linInterpolate( l_positionDelta, obj.alignmentTipTransitDistance, obj.initialTipPose(2), obj.alignmentTipPose(2));
                l_thetaTarget = obj.linInterpolate( l_positionDelta, obj.alignmentTipTransitDistance, obj.initialTipPose(3), obj.alignmentTipPose(3));
                plot(l_xTarget, l_yTarget, 'og', 'MarkerSize', 10);
                
                if(isempty(obj.arm2D.kTarget))
                    l_kGuess = 0.1*randn(1, 6);
                else
                    l_kGuess = obj.arm2D.kTarget;
                end
                
                [l_kTarget] = obj.arm2D.inverseKinematics(l_xTarget, l_yTarget, l_thetaTarget, l_kGuess);
                obj.arm2D.setTargetCurvatures(l_kTarget);
                obj.arm2D.gripper2D.setTargetCurvatures(0);
                obj.arm2D.actuate();
            end
        end
        %Determine and drive arm tip to an advanced pose alongside object
        function advanceArmTip(obj)
                        
            if( obj.advancementPathProfileComputed == 0 ) % compute alignement path profile
                d = norm([obj.alignmentTipPose(1:2)-obj.advancedTipPose(1:2)], 2);
                obj.advancementPathProfile = obj.generateVelocityProfile( d );
                obj.advancementTipTransitDistance = d;
                obj.advancementPathProfileComputed = 1;
            end
            
            %%%%%%%%%%%%%%%%% determine end effector pose %%%%%%%%%%%%%%%%%
            l_k = obj.arm2D.kMeas;
            l_L = obj.arm2D.arcLenMeas;
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
                l_tCurrent = toc(obj.planTime);
                l_positionDelta = obj.getPositionDelta(obj.advancementPathProfile, l_tCurrent);
                
                l_xTarget = obj.linInterpolate( l_positionDelta, obj.advancementTipTransitDistance, obj.alignmentTipPose(1), obj.advancedTipPose(1));
                l_yTarget = obj.linInterpolate( l_positionDelta, obj.advancementTipTransitDistance, obj.alignmentTipPose(2), obj.advancedTipPose(2));
                l_thetaTarget = obj.linInterpolate( l_positionDelta, obj.advancementTipTransitDistance, obj.alignmentTipPose(3), obj.advancedTipPose(3));
                plot(l_xTarget, l_yTarget, 'og', 'MarkerSize', 10);
                
                l_kGuess = obj.arm2D.kTarget;
                [l_kTarget] = obj.arm2D.inverseKinematics(l_xTarget, l_yTarget, l_thetaTarget, l_kGuess);
                obj.arm2D.setTargetCurvatures(l_kTarget);
                obj.arm2D.gripper2D.setTargetCurvatures(0);
                obj.arm2D.actuate();
            end
        end
        %Actuate the gripper to grasp object
        function graspObject(obj)
            %%%%%%%%%%%%%%%% is gripper at grasp curvature? %%%%%%%%%%%%%%%
            obj.arm2D.gripper2D.setTargetCurvatures( obj.arm2D.gripper2D.dims.kMax );
            obj.arm2D.actuate();
            
%             %%%%%%%%%%%%%%%%% if it is at grasp curvature %%%%%%%%%%%%%%%%%
%             if( norm( l_k - obj.graspCurvature, 2 ) <= 1 )
%                 obj.state = 5; %then move to next planning state
%                 display('Gripper has grasped');
%                 obj.planTime = tic; % get current time
%                 
%                 %%%%%%%%%%%% otherwise advance gripper curvature %%%%%%%%%%%%%%
%             else
%                 if( toc(obj.planTime) <= obj.graspTime )
%                     l_kInit = 0; % <---- Unactauted gripper curvature
%                     l_kTarget = obj.linInterpolate(toc(obj.planTime), obj.graspTime, l_kInit, obj.graspCurvature);
%                     obj.arm2D.gripper2D.setTargetCurvatures(l_kTarget);
%                     obj.arm2D.actuate();
%                 else
%                     obj.arm2D.gripper2D.setTargetCurvatures( obj.graspCurvature );
%                     obj.arm2D.actuate();
%                 end
%             end
        end
        %An interporlation function
        function valBetween = linInterpolate(obj, distanceCurrent, distanceMax, posInitial, posFinal)
            valBetween = posInitial + (posFinal - posInitial)*(distanceCurrent)/(distanceMax);
        end
        %Generate a feasible Cartestian velocity profile given a desired distance to move
        function velocityProfile = generateVelocityProfile(obj, d )
            
            if( d < (obj.vMax^2/obj.aMax) )
                velocity = sqrt(d*obj.aMax)-0.0001;
            else
                velocity = obj.vMax;
            end
            
            t1 = velocity/obj.aMax;
            t2 = (d - (velocity*t1))/velocity;
            tf = 2*t1+t2;
            
            velocityProfile = PlannerGrasp.firstOrderHold( [0, t1, t1+t2, tf, tf+0.1], [0, velocity, velocity, 0, 0] );
        end
        %Get a feasible Cartesian position delta based on a velcoity
        function PositionDelta = getPositionDelta(obj, velocityProfile, tCurrent)
            %tCurrent is the time along a linear path starting at t=0
            PositionDelta = integral( @(x)ppval(velocityProfile,x), 0, (tCurrent + 0.050) );
        end
        
    end
    
    methods(Static)
        function ypp = firstOrderHold(t0,y0)
            % First-ORDER-HOLD
            %   Creates a pp form (piecewise polynomial) of order 1 which implements a
            %   first-order hold.  Use ppval, ppval_safe, or fnval to evaluate it.
            %   Whatever the size of y0, the last dimension is paired up with time.
            
            D = size(y0); L = D(end)-1; D = D(1:(end-1));
            if (length(t0)~=L+1) error('t0 and y0 do not match'); end
            
            y0 = reshape(y0,[],L+1);
            t0 = reshape(t0,1,L+1);
            coefs(:,2) = reshape(y0(:,1:L),[],1);
            coefs(:,1) = reshape(diff(y0,1,2)./repmat(diff(t0),[size(y0,1) 1]),[],1);
            ypp = mkpp(t0,coefs,D);
        end
    end
end

