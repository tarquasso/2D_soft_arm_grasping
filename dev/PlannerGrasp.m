classdef PlannerGrasp < handle
    
    properties
        arm2D;
        roundObject;
        framePeriod;
        plannerType;
        trajGen;
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
        
        vMax;
        aMax;
        
        %ArcSpace Planner
        transitDistInc;
        posMoveEpsilon;
        rotMovEpsilon;
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
        waitTimeForGrasp;
        binTrajGenerated;
        xBin;
        yBin;
    end
    
    methods
        %Constructor
        function obj = PlannerGrasp(plannerTypeInput,arm2DHand,roundObjectHand,trajGenHand,framePeriod)
            obj.arm2D =  arm2DHand;
            obj.roundObject = roundObjectHand;
            obj.trajGen = trajGenHand;
            obj.framePeriod = framePeriod;
            %ArcSpace Planner
            obj.transitDistInc = 0.03;
            obj.posMoveEpsilon = 0.02;
            obj.rotMovEpsilon = 7.5*(180/3.14159);
            obj.nMov = 1;
            obj.arcSpacePlanDone = false;
            obj.moveTo = 1;
            obj.kOff1 = 3;
            obj.binTrajGenerated = 0;
            obj.xBin = 0; 
            obj.yBin = sum(obj.arm2D.dims.lengths);
            
            %Cartesian Planner
            obj.state = 1;
            obj.objInRightPosition = false;
            obj.previousObjPosition = [0; 0];
            obj.changeInObjPositionThreshold = 0.005; % half of a centimeter
            obj.timeAtObjPosition = 0.0;
            obj.timeAtObjPositionThreshold = 2.0;
            
            obj.initialTipPoseComputed = 0;
            obj.initialTipPose = zeros(3,1);
            
            obj.alignmentTipPoseComputed = 0;
            obj.alignmentTipPose = zeros(3,1);
            obj.advancedTipPose = zeros(3,1);
            obj.graspCurvature = obj.arm2D.gripper2D.dims.kMax;
            
            obj.alignmentPathProfileComputed = 0;
            obj.advancementPathProfileComputed = 0;
            obj.graspTime = 3.0;
            
            obj.vMax = 0.002;
            obj.aMax = 0.001;
            
            obj.plannerFree = 'true';
            obj.plannerType = plannerTypeInput;
            
            obj.trajGenerated = 0;
            obj.startedToSettle = false;
            obj.startedToGrasp = false;
            obj.waitTimeForSettle = 1.0;%s
            obj.waitTimeForGrasp = 1.5; %s
        end
        %Destructor
        function delete(obj)
        end
        %Plan dispatcher
        function result = plan(obj)
            result = 0;
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
                    case 4
                        obj.checkArmSettled();
                    case 5 % object is NOT grasped
                        obj.graspObject();
                    case 6 % move object to bin
                        obj.moveToBin();
                    otherwise
                        display('[PlannerGrasp] Plan fully executed');
                        result = 1;
                        return;
                       
                end
                obj.plannerFree = 'true';
            else
                obj.arm2D.actuate();
            end
            
            
        end
        
        function moveToBin(obj)
            % Check if object is at bin
            [xTipCur, yTipCur, thetaTipCur] = ...
                obj.arm2D.recursiveForwardKinematics(obj.arm2D.kMeas,...
                obj.arm2D.dims.S, obj.arm2D.arcLenMeas(obj.arm2D.dims.S));
            
            if(  norm(([obj.xBin; obj.yBin]-[xTipCur; yTipCur]),2) <= obj.posMoveEpsilon )
                
                if( obj.binTrajGenerated == 0 )
                    
                    display('[ArcSpacePlanner] Moving to bin');
                    
                    %generate realizable trajectory
                    l_kInitial = double(obj.arm2D.kMeas); % config traj start values
                    % find k Target using IK
                    l_thetaTarget = pi/2;
                    l_kGuess = 0.01*ones(1, obj.arm2D.dims.S); %approx zeros curvatures
                    [l_kTarget] = obj.arm2D.inverseKinematics(obj.xBin, obj.yBin, l_thetaTarget, l_kGuess);
                    % request multiple configuration velocity trajs
                    [obj.curvatureProfiles, obj.trajectoryEndTime ] = obj.trajGen.generateMultipleVelocityProfiles(obj, l_kInitial, l_kTarget );
                    % store start and end values
                    obj.kInit = l_kInitial;
                    obj.kGoal = l_kTarget;
                    
                    obj.planTime = tic; % get current time
                    obj.binTrajGenerated = 1;
                end
                
                %generate intermediate k
                [l_kIntermediate] = obj.trajGen.generateMultplePositionDeltas(obj.curvatureProfiles, toc(obj.planTime), obj.kInit);
                %send intermediate k
                obj.arm2D.setTargetCurvatures(l_kIntermediate);
                % actuate the arm and the gripper
                obj.arm2D.actuate();
            else
                %arrived at final bin
                display('[ArcSpacePlanner] Object is at bin');
                %send k to measured to stop arm
                obj.arm2D.setTargetCurvatures(obj.arm2D.kMeas);
                %set gripper curvature to 0
                obj.arm2D.gripper2D.setTargetCurvatures(0);
                % actuate the arm and the gripper
                obj.arm2D.actuate();
                %set state of planner to step 7, which is to finish
                obj.state = 7;
                % get current time
                obj.planTime = tic;
            end
            
        end
        
        function arcSpacePlan(obj)
            if(obj.arcSpacePlanDone == false)
                
                % Input is initial end effector pose and we need round object
                % radius and round object center position
                finalRadius = obj.arm2D.gripper2D.dims.offCenter+obj.roundObject.r;
                [xTipCur, yTipCur, thetaTipCur] = ...
                    obj.arm2D.recursiveForwardKinematics(obj.arm2D.kMeas,...
                    obj.arm2D.dims.S, obj.arm2D.arcLenMeas(obj.arm2D.dims.S));
                tipToObject = norm([xTipCur; yTipCur] - [obj.roundObject.x; obj.roundObject.y],2);
                transitDist = tipToObject - finalRadius;
                
                obj.nMov = floor(transitDist/obj.transitDistInc);
                allRadii = zeros(1,obj.nMov);
                
                obj.kOptimal = zeros(obj.nMov,obj.arm2D.dims.S);
                
                obj.gammaOptimal = zeros(obj.nMov,1);
                obj.tipOptimal = zeros(obj.nMov,3);
                
                % For debugging purposes - plot connecting line
                plot([xTipCur, obj.roundObject.x], [yTipCur, obj.roundObject.y], 'g');
                
                for i=1:1:obj.nMov
                    allRadii(1,i) = tipToObject-i*transitDist/obj.nMov;
                    if (i==1)
                        %define gammaGuess as the initial theta plus 90 degrees
                        l_gammaGuess = pi/2; % + obj.arm2D.thetaMeas(end);
                        % k guess is initial kmeasured
                        l_kGuess = obj.arm2D.kMeas;
                        %find optimal gamma and k using last measured theta and k
                        [obj.gammaOptimal(i,:),obj.kOptimal(i,:)] = ...
                            obj.findOptimalK(allRadii(i),l_gammaGuess,l_kGuess);
                    else
                        %find optimal k using last found optimal
                        %theta and k
                        l_gammaGuess = obj.gammaOptimal(i-1,:);
                        l_kGuess = obj.kOptimal(i-1,:);
                        [obj.gammaOptimal(i,:),obj.kOptimal(i,:)] = ...
                            obj.findOptimalK(allRadii(i),l_gammaGuess,l_kGuess);
                    end
                    %calculate tip vector
                    l_i = obj.arm2D.dims.S;
                    l_s = obj.arm2D.arcLenMeas(l_i);
                    
                    [obj.tipOptimal(i,1), obj.tipOptimal(i,2), obj.tipOptimal(i,3)] = ...
                        obj.arm2D.recursiveForwardKinematics( obj.kOptimal(i,:), l_i, l_s );
                    
                    % For debugging purposes
                    obj.plotArc(allRadii(1,i));
                    plot( obj.tipOptimal(i,1), obj.tipOptimal(i,2), 'or', 'MarkerSize', 10 );
                    obj.arm2D.plotArmMeasToHandle(obj.kOptimal(i,:));
                    drawnow;
                end

                obj.arcSpacePlanDone = true;
                
            else
                %compute current tip pose from measure curvatures
                l_i = obj.arm2D.dims.S;
                l_s = obj.arm2D.arcLenMeas(l_i);
                
                [l_tipX, l_tipY, l_tipTheta] = ...
                    obj.arm2D.recursiveForwardKinematics( obj.arm2D.kMeas, l_i, l_s );
                
                %check if distance to target radius is smaller than epsilon
                if( norm( [l_tipX, l_tipY] - obj.tipOptimal(obj.moveTo,1:2),2) <= obj.posMoveEpsilon ...
                        && norm( l_tipTheta - obj.tipOptimal(obj.moveTo,3), 2) <= obj.rotMovEpsilon )
                    %increment move to by 1
                    obj.moveTo = obj.moveTo +1;
                    obj.trajGenerated = 0; 
                end
                
                % send target curvatures until final pose is achieved
                if(obj.moveTo <= obj.nMov)
                    
                    if( obj.trajGenerated == 0 )
                        
                        display(['Moving to arc:',num2str(obj.moveTo)]);
                        
                        %generate realizable trajectory
                        l_kInitial = double(obj.arm2D.kMeas); % config traj start values
                        l_kTarget = obj.kOptimal(obj.moveTo,:); % config traj end values
                        % request multiple configuration velocity trajs
                        [obj.curvatureProfiles, obj.trajectoryEndTime ] = obj.trajGen.generateMultipleVelocityProfiles(obj, l_kInitial, l_kTarget );
                        % store start and end values
                        obj.kInit = l_kInitial;
                        obj.kGoal = l_kTarget;
                        
                        obj.planTime = tic; % get current time
                        obj.trajGenerated = 1;
                    end
                    
                    %generate intermediate k
                    [l_kIntermediate] = obj.trajGen.generateMultplePositionDeltas(obj.curvatureProfiles, toc(obj.planTime), obj.kInit);
                    %send intermediate k
                    obj.arm2D.setTargetCurvatures(l_kIntermediate);
                    %set gripper curvature to 0
                    obj.arm2D.gripper2D.setTargetCurvatures(0);
                    % actuate the arm and the gripper
                    obj.arm2D.actuate();
                else % obj.moveTo > obj.nMov
                    %arrived at final pose
                    display('[ArcSpacePlanner] Arm is at final pose');
                    %reset moveTo variable back to '1' for next time
                    %obj.moveTo = 1;
                    %reset arcSpace Planner Done to false for next time
                    %obj.arcSpacePlanDone = false;
                    %set state of planner to step 4, which is to grasp the object
                    obj.state = 4;
                    % get current time
                    obj.planTime = tic;
                end
            end
        end
        %Determine whether or not arm has settled before object
        function checkArmSettled(obj)
            
            % Target = Measured for arm and gripper
            
            obj.arm2D.setTargetCurvatures(obj.arm2D.kMeas); %set target to be equal to be the measured value
            obj.arm2D.gripper2D.setTargetCurvatures(obj.arm2D.gripper2D.kMeas); %set target to be equal to be the measured value
            obj.arm2D.actuate();
            
            if(obj.startedToSettle==false)
                display(['[ArcSpacePlanner] Waiting ',num2str(obj.waitTimeForSettle),' s for arm to settle']);
                obj.planTime = tic;
                obj.startedToSettle = true;
            else
                if(toc(obj.planTime) > obj.waitTimeForSettle )
                    obj.startedToSettle = false;
                    obj.state = 5; % grasp
                end              
            end           
        end
        function plotArc(obj, r)
            delta_angle = 0.01;
            
            ang = 0:delta_angle:2*pi;
            xp = r*cos(ang);
            yp = r*sin(ang);
            h = plot(obj.roundObject.x+xp,obj.roundObject.y+yp,'-g');
        end
        
        function [gammaOptimal,kOptimal] = findOptimalK(obj,approachRadius,gammaGuess,kGuess)
            % Find local optimal curvatures that satisfy constraints for
            % cylce i
            
            %% COMPLETE minus debugging
            A = []; % A, b, Aeq, and beq all used for linear constraints, intentionally empty here
            b = [];
            Aeq = [];
            beq = [];
            
            % lower and upper bound on decision parameters [theta, curvatures]
            % theta = end effector angle - pi/2
            lb = [0, obj.arm2D.dims.kMin];
            ub = [2*pi, obj.arm2D.dims.kMax];
            
            guessParameters = [gammaGuess, kGuess];
            
            options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3,...
                'TolX', 1e-6,'GradObj','on', 'GradConstr', 'off','Display','off');
            
            optimalParameters = fmincon(@cost,guessParameters,A,b,Aeq,beq,...
                lb,ub,@noncon,options);
            
            gammaOptimal = optimalParameters(1);
            kOptimal = optimalParameters(2:end);
            
            function [c,ceq] = noncon(parametersCurrent)
                gamma = parametersCurrent(1);
                k = parametersCurrent(2:end);
                
                xTarget = (obj.roundObject.x) + (approachRadius)*cos(gamma);
                yTarget = (obj.roundObject.y) + (approachRadius)*sin(gamma);
                thetaTarget = pi/2 + gamma;
                
                % visualization for debugging
                plot(xTarget, yTarget, 'og', 'MarkerSize', 10 );
                
                ceq = zeros(1,3);        % nonlinear equalitity constraints
                
                l_i = obj.arm2D.dims.S;
                l_s = obj.arm2D.arcLenMeas(l_i);
                
                % End effector nonlinear equality constraints <- enforce
                % tip position
                [xTipCurrent, yTipCurrent, thetaTipCurrent] = obj.arm2D.recursiveForwardKinematics(k, l_i, l_s);
                
                xToolCurent = xTipCurrent + obj.roundObject.r*cos(thetaTipCurrent);
                yToolCurent = yTipCurrent + obj.roundObject.r*sin(thetaTipCurrent);
                
                ceq(1) = xToolCurent- xTarget;
                ceq(2) = yToolCurent - yTarget;
                ceq(3) = thetaTipCurrent - thetaTarget;
                
                % TODO: rewrite recursive FK to return intermediate
                % endpoints so that we dont have to call it twice
                
                %  nonlinear inequality constraints on segment N-1 <- ensure wrist object
                %  avoidance
                c = [];
                %[xWristCurrent, yWristCurrent, thetaWristCurrent] = obj.arm2D.recursiveForwardKinematics(k, l_i, l_s-1);
                %objectCenter = [obj.roundObject.x; obj.roundObject.y];
                %c = obj.roundObject.r - norm([xWristCurrent; yWristCurrent] - objectCenter, 2);
            end
            
            function [E_tot, g] = cost(parametersCurrent)
                k = parametersCurrent(2:end);
                R = [1, 0.1, 0.1, 0.1, 0.1, 0.1];
                E_tot = sum( [R(1)*(k(1)+obj.kOff1), R(2)*k(2), R(3)*k(3), R(4)*k(4), R(5)*k(5), R(6)*k(6)].^2);
                g = 2.*[R(1)*(k(1)+obj.kOff1), R(2)*k(2), R(3)*k(3), R(4)*k(4), R(5)*k(5), R(6)*k(6)];
                g = [0, g];
            end
        end
        %Determine whether or not and object has been placed
        function checkObjPlacement(obj)
            
            persistent counter;
            
            % Target = Measured for arm and gripper
            obj.arm2D.setTargetCurvatures(obj.arm2D.kMeas); %set target to be equal to be the measured value
            obj.arm2D.gripper2D.setTargetCurvatures(obj.arm2D.gripper2D.kMeas); %set target to be equal to 0
            
            l_currentObjPosition = [obj.roundObject.x; obj.roundObject.y];
            
            if( norm(obj.previousObjPosition,2) >= 0) % prev not at default location
                
                l_changeInObjPosition = norm( l_currentObjPosition - obj.previousObjPosition, 2);
                
                if( l_changeInObjPosition <= obj.changeInObjPositionThreshold )
                    if(obj.objInRightPosition == false)
                        fprintf('Waiting for object to settle\n');
                        obj.objInRightPosition = true;
                        counter = 0;
                    end
                    if( obj.timeAtObjPosition >= obj.timeAtObjPositionThreshold ) % has it been enough time?
                        fprintf('\n');
                        display('Object is placed')
                        obj.state = 2;
                    else
                        fprintf('.');
                        if (counter > 30)
                            fprintf('\n');
                            counter = 0;
                        end
                        counter = counter +1;
                    end
                    obj.timeAtObjPosition = obj.timeAtObjPosition + toc(obj.planTime);
                else
                    obj.objInRightPosition = false;
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
            if(obj.startedToGrasp == false)
                obj.planTime = tic;
                obj.startedToGrasp = true;
                obj.arm2D.gripper2D.setTargetCurvatures( obj.arm2D.gripper2D.dims.kMax );
                obj.arm2D.actuate();        
            else
                if(toc(obj.planTime) > obj.waitTimeForGrasp )
                    obj.startedToGrasp = false;
                    obj.state = 6; % next state
                end              
            end    
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
            PositionDelta = integral( @(x)ppval(velocityProfile,x), 0, (tCurrent + 0.025) );
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

