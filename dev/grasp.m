function [] = grasp( )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize arm %%%%%%%%%%%%%%%%%%%%%%%%%
    k_init = 5*randn(6,1);
    ARM = Arm2D();
    ARM.setMeasuredLengths(2.37*0.0254*ones(6,1));
    ARM.setMeasuredCurvatures(k_init);
    ARM.setTargetCurvatures(k_init);
    ARM.gripper2D.setMeasuredLengths(4.0*0.0254);
    ARM.gripper2D.setMeasuredCurvatures(1.0);
    ARM.gripper2D.setTargetCurvatures(1.0);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Object %%%%%%%%%%%%%%%%%%%%%%%%
    OBJECT = RoundObject();
    obj_x = 0.15;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_y = 0.325;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    obj_r = 0.025;         % r is the radius of the circle             <--- MEASURE FROM MOCAP INITIAL FRAME
    OBJECT.setMeasuredState(obj_x, obj_y, obj_r);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Planner %%%%%%%%%%%%%%%%%%%%%%%
    PLANNER = PlannerGrasp(ARM, OBJECT);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
    figure;
    hold on;
    plot([0,0],[-1,1], '-k');
    plot([-1,1],[0,0], '-k');
    
    armPlotHandle = ARM.plotArmToHandle();
    objectPlotHandle = OBJECT.plotObjectToHandle();
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% Start Control Loop %%%%%%%%%%%%%%%%%%%%%%%
    t = timer;
    t.Period = 0.25;
    t.TimerFcn = @(myTimerObj, thisEvent)TimerFunction;
    t.StartFcn = @(myTimerObj, thisEvent)StartFunction;
    t.ExecutionMode = 'fixedRate';
    start(t)
    pause(20) % <----- Total experiment time
    stop(t)
    delete(t)
    
    function TimerFunction

        PLANNER.plan();
        ARM.setMeasuredCurvatures(ARM.kTarget);
        ARM.gripper2D.setMeasuredCurvatures(ARM.gripper2D.kTarget);
        delete(armPlotHandle);
        armPlotHandle = ARM.plotArmToHandle();
    end

    function StartFunction( )

    end
end





 
   
    
    
    
    
    