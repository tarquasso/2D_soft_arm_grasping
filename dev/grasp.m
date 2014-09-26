function [] = grasp( )

rpttype = 'extended';
%% %%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize arm %%%%%%%%%%%%%%%%%%%%%%%%%

ARM = Arm2D(ExpTypes.Simulation);
%ARM.setMeasuredLengths(2.47*0.0254*ones(1, 6));
%ARM.setMeasuredCurvatures(k_init);
%ARM.setTargetCurvatures(k_init);
%ARM.gripper2D.setMeasuredLengths(4.0*0.0254);
%ARM.gripper2D.setMeasuredCurvatures(1.0);
%ARM.gripper2D.setTargetCurvatures(1.0);

%% %%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Object %%%%%%%%%%%%%%%%%%%%%%%%
xMax = -0.10;
xMin = -0.20;
yMax = 0.30;
yMin = 0.20;

%     xMax = -0.05;
%     xMin = -0.25;
%     yMax = 0.38;
%     yMin = 0.10;

OBJECT = RoundObject();
obj_x = (xMax-xMin).*rand(1)+xMin;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
obj_y = (yMax-yMin).*rand(1)+yMin;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
OBJECT.setMeasuredState(obj_x, obj_y);

%obj.sensor = Sensor(ARM,OBJECT);

%% %%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Planner %%%%%%%%%%%%%%%%%%%%%%%
framePeriod = 0.1; %s
TRAJ = TrajGen();
PLANNER = PlannerGrasp(PlannerTypes.ArcSpacePlanner, ARM, OBJECT, TRAJ, framePeriod);

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
figure;
hold on;
plot([0,0],[-1,1], '-k');
plot([-1,1],[0,0], '-k');

plot([xMin,xMin],[yMin,yMax], '-k');
plot([xMax,xMax],[yMin,yMax], '-k');
plot([xMin,xMax],[yMin,yMin], '-k');
plot([xMin,xMax],[yMax,yMax], '-k');

armPlotHandle = ARM.plotArmMeasToHandle(ARM.kMeas);
objectPlotHandle = OBJECT.plotObjectToHandle();
expTime = 12;
expTimer = tic;
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Start Control Loop %%%%%%%%%%%%%%%%%%%%%%%
t = timer;
t.Period = framePeriod;
t.TimerFcn = @(myTimerObj, thisEvent)TimerFunction;
t.StartFcn = @(myTimerObj, thisEvent)StartFunction;
t.ErrorFcn = @(myTimerObj, thisEvent)ErrorFunction;
t.ExecutionMode = 'fixedRate';
broken = false;
start(t)
while(toc(expTimer) < expTime || broken ==false)
    pause(0.5)
end% <----- Total experiment time
stop(t)
delete(t)

    function TimerFunction
        
        try
            if (broken == false)
                ARM.setMeasuredCurvatures(ARM.kTarget);
                
                ARM.setMeasuredLengths(2.47*0.0254*ones(1, 6));
                
                ARM.gripper2D.setMeasuredCurvatures(ARM.gripper2D.kTarget);
                
                % ARM.gripper2D.setMeasuredLengths(ARM.gripper2D.kTarget);
                % update kMeas, arcLens, thetaMeas, segPos
                ARM.segPos2D = 10*rand(2,7);
                ARM.setMeasuredTheta(rand(1,6));
                l_result = PLANNER.plan();
                
                delete(armPlotHandle);
                
                armPlotHandle = ARM.plotArmMeasToHandle(ARM.kTarget);                
                %toc(expTimer)
                if(l_result == 1)
                    broken = true;
                end
            end
        catch exc
            getReport(exc, rpttype)
            stop(t)
        end
    end

    function StartFunction( )
        
    end
    function ErrorFunction( )
        broken = true;
    end

end











