function [] = grasp( )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize arm %%%%%%%%%%%%%%%%%%%%%%%%%
    k_init = 1*randn(6,1);
    ARM = Arm2D();
    ARM.setMeasuredLengths(2.37*0.0254*ones(6,1));
    ARM.setMeasuredCurvatures(k_init);
    ARM.setTargetCurvatures(k_init);
    ARM.gripper2D.setMeasuredLengths(4.0*0.0254);
    ARM.gripper2D.setMeasuredCurvatures(1.0);
    ARM.gripper2D.setTargetCurvatures(1.0);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Object %%%%%%%%%%%%%%%%%%%%%%%%
    xMax = 0.15;
    xMin = -0.15;
    yMax = 0.35;
    yMin = 0.30;
    
    OBJECT = RoundObject();
    obj_x = (xMax-xMin).*rand(1)+xMin;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_y = (yMax-yMin).*rand(1)+yMin;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_r = 0.025;         % r is the radius of the circle             <--- MEASURE FROM MOCAP INITIAL FRAME
    OBJECT.setMeasuredState(obj_x, obj_y, obj_r);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Planner %%%%%%%%%%%%%%%%%%%%%%%
    framePeriod = 0.1;
    PLANNER = PlannerGrasp(ARM, OBJECT, framePeriod);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
    figure;
    hold on;
    plot([0,0],[-1,1], '-k');
    plot([-1,1],[0,0], '-k');
    
    plot([xMin,xMin],[yMin,yMax], '-k');
    plot([xMax,xMax],[yMin,yMax], '-k');
    plot([xMin,xMax],[yMin,yMin], '-k');
    plot([xMin,xMax],[yMax,yMax], '-k');
    
    armPlotHandle = ARM.plotArmToHandle();
    objectPlotHandle = OBJECT.plotObjectToHandle();
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Start Control Loop %%%%%%%%%%%%%%%%%%%%%%%
    t = timer;
    t.Period = framePeriod;
    t.TimerFcn = @(myTimerObj, thisEvent)TimerFunction;
    t.StartFcn = @(myTimerObj, thisEvent)StartFunction;
    t.ExecutionMode = 'fixedRate';
    start(t)
    pause(20) % <----- Total experiment time
    stop(t)
    delete(t)
    
    function TimerFunction
        ARM.setMeasuredCurvatures(ARM.kTarget);
        ARM.gripper2D.setMeasuredCurvatures(ARM.gripper2D.kTarget);
        PLANNER.plan();
        delete(armPlotHandle);
        armPlotHandle = ARM.plotArmToHandle();
    end

    function StartFunction( )

    end
end





 
   
    
    
    
    
    