function [] = grasp( )

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% initializations %%%%%%%%%%%%%%%%%%%%%%%%%
    
    ARM = arm();
    GRIPPER = gripper();
    
    ARM.theta0 = pi/2;              % is the current/measured initial orientation of the first segment
    ARM.L = 2.37*0.0254*ones(6,1);  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
    ARM.N = 6;                      % is the total number of arm segments
    ARM.k_min = -20;                % minimum allowable curvature
    ARM.k_max = 20;                 % maximum allowable curvature
    GRIPPER.L = 4.15*0.0254;        % length of gripper
    GRIPPER.k = 0.1;                % initial gripper curvature
    
    k_init = 1*ones(6,1);       	% initial measured  arm curvatures <--- MEASURE FROM MOCAP INITIAL FRAME
    max_iter = 5;                   % number of advancement steps the gripper makes
    settling_time = 1.0;            % <--- SETTLING TIME, TO BE DETERMINED EXPERIMENTALLY
                                    % TODO: a better idea is to measure when the
                                    % manipulator has settled.
                                    
    [init_x, init_y, init_theta] = recursive_forward_kinematics(k_init, ARM.theta0, ARM.L, ARM.N, ARM.N, ARM.L(ARM.N) );
                              
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% plot the object %%%%%%%%%%%%%%%%%%%%%%%%%%
    object_x = 0.15;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    object_y = 0.30;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
    object_r = 0.025;         % r is the radius of the circle             <--- MEASURE FROM MOCAP INITIAL FRAME
    
    figure;
    hold on;
    plot([0,0],[-1,1], '-k');
    plot([-1,1],[0,0], '-k');
    chain_handle = PlotChain(k_init, ARM, GRIPPER);

    circle(object_x, object_y, object_r);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%% compute target tip position %%%%%%%%%%%%%%
    D = sqrt(object_x^2 + object_y^2);               % distance from group to object center
    object_angle = atan2(object_y, object_x)-pi/2;   % angle between ground and object center 
    d = 2*object_r;                                  % perpendicular offset from object
    l = GRIPPER.L;                                   % parallel offset from object
    v = 0;                                           % parallel advancement amount

    target_x = d*cos(object_angle) - (D - l - object_r)*sin(object_angle) - v*sin(object_angle);
    target_y = (D - l - object_r)*cos(object_angle) + v*cos(object_angle) + d*sin(object_angle);
    target_theta = object_angle + ARM.theta0;

    plot(target_x, target_y, 'ok', 'MarkerSize', 10);

    %%%%%%%%%%%%%%%%%%%%% find path to target tip position %%%%%%%%%%%%%%%%
    transit_time = 5.0;
    k_target = k_init;
    
    tic;
    while( toc <= transit_time )
        x = linInterpolate(toc, transit_time, init_x, target_x);
        y = linInterpolate(toc, transit_time, init_y, target_y);
        theta = linInterpolate(toc, transit_time, init_theta, target_theta);
        
        [k_target] = inverseKinematics(x, y, theta, k_target, ARM);
        
        delete(chain_handle);
        chain_handle = PlotChain(k_target, ARM, GRIPPER); 
        pause(0.1);
    end
          
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%5 Advance the gripper %%%%%%%%%%%%%%%%%%%%%
    for i=0:max_iter
        %%%%%%%%%%%%%%%%%%%%%%%%%%%% compute target tip position %%%%%%%%%%%%%%
        v = (i/max_iter)*l/1.5;                      % parallel advancement amount

        target_x = d*cos(object_angle) - (D - l - object_r)*sin(object_angle) - v*sin(object_angle);
        target_y = (D - l - object_r)*cos(object_angle) + v*cos(object_angle) + d*sin(object_angle);

        plot(target_x, target_y, 'ok', 'MarkerSize', 10);

        %%%%%%%%%%%%%%%%%%%%% find IK solution to get to target tip position %%
        [k_target] = inverseKinematics(target_x, target_y, target_theta, k_target, ARM);
        delete(chain_handle);
        chain_handle = PlotChain(k_target, ARM, GRIPPER);   
        pause(settling_time);
    end
   
    % actuate the gripper 
    delete(chain_handle);
    GRIPPER.k = 1/object_r;
    chain_handle = PlotChain(k_target, ARM, GRIPPER);   
    pause(1.0);
   

end

function val = linInterpolate(t, t_max, val_0, val_1)
    val = val_0 + (val_1 - val_0)*(t)/(t_max);
end
   
    function circle(x,y,r)
        %0.01 is the angle step
        ang=0:0.01:2*pi; 
        xp=r*cos(ang);
        yp=r*sin(ang);
        plot(x+xp,y+yp,'-b');
    end
    
    function [k_target] = inverseKinematics(target_x, target_y, target_theta, k_guess, ARM)
    
        A = [];
        b = [];
        Aeq = [];
        beq = [];
        lb = ARM.k_min*ones(6,1);
        ub = ARM.k_max*ones(6,1);
    
        options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3, 'TolX', 1e-6,'GradObj','on', 'GradConstr', 'off');
        k_target = fmincon(@cost,k_guess,A,b,Aeq,beq,lb,ub,@noncon,options); 
        
        function [c,ceq] = noncon(k)
            c = [];                  % nonlinear inequality constraints
            ceq = zeros(1,3);        % nonlinear equalitity constraints
            [measured_x, measured_y, measured_theta] = recursive_forward_kinematics(k, ARM.theta0, ARM.L, ARM.N, ARM.N, ARM.L(ARM.N));
            ceq(1) = measured_x - target_x;
            ceq(2) = measured_y - target_y;
            ceq(3) = measured_theta - target_theta;
        end
    
        function [E_tot, g] = cost(k)
            E_tot = sum(k.^2);
            g = 2.*k;
        end
    
    end
    
    
    
    
   
    
    
    
    
    