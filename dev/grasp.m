function [] = grasp( )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize arm %%%%%%%%%%%%%%%%%%%%%%%%%
    k_init = 0.1*randn(1, 6);
    ARM = Arm2D();
    ARM.setMeasuredLengths(2.47*0.0254*ones(1, 6));
    ARM.setMeasuredCurvatures(k_init);
    ARM.setTargetCurvatures(k_init);
    ARM.gripper2D.setMeasuredLengths(4.0*0.0254);
    ARM.gripper2D.setMeasuredCurvatures(1.0);
    ARM.gripper2D.setTargetCurvatures(1.0);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Object %%%%%%%%%%%%%%%%%%%%%%%%
    xMax = -0.10;
    xMin = -0.20;
    yMax = 0.30;
    yMin = 0.20;
    
    OBJECT = RoundObject();
    obj_x = (xMax-xMin).*rand(1)+xMin;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_y = (yMax-yMin).*rand(1)+yMin;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_r = 0.025;         % r is the radius of the circle             <--- MEASURE FROM MOCAP INITIAL FRAME
    OBJECT.setMeasuredState(obj_x, obj_y);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Planner %%%%%%%%%%%%%%%%%%%%%%%
    framePeriod = 0.1;
    PLANNER = PlannerGrasp(PlannerTypes.ArcSpacePlanner, ARM, OBJECT, framePeriod);
    
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Start Control Loop %%%%%%%%%%%%%%%%%%%%%%%
    t = timer;
    t.Period = framePeriod;
    t.TimerFcn = @(myTimerObj, thisEvent)TimerFunction;
    t.StartFcn = @(myTimerObj, thisEvent)StartFunction;
    t.ExecutionMode = 'fixedRate';
    start(t)
    pause(12) % <----- Total experiment time
    stop(t)
    delete(t)
    
    function TimerFunction
        ARM.setMeasuredCurvatures(ARM.kTarget);
        ARM.setMeasuredLengths(2.47*0.0254*ones(1, 6));
        ARM.gripper2D.setMeasuredCurvatures(ARM.gripper2D.kTarget);
        %ARM.gripper2D.setMeasuredLengths(ARM.gripper2D.kTarget);
        % update kMeas, arcLens, thetaMeas, segPos
        PLANNER.plan();
        delete(armPlotHandle);
        armPlotHandle = ARM.plotArmMeasToHandle(ARM.kTarget);
    end

    function StartFunction( )

    end
end





 
   
    
    
    
    
    