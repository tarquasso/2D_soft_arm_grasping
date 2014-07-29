function [] = Optimization_IK_experiment
    %% Simulation settings
    close all
    %clear all
    clc
    % Model of the arm
    arm_model = 'B';


    %% Startup
    % Add folders to the path
    addpath('tracker/data_structures', ...
            'tracker/geometry', ...
            'tracker/kinematics', ...
            'tracker/peripherals/curvature_controller',  ...
            'tracker/peripherals/tracking_tools', ...
            'tracker/setup', ...
            'tracker/various')

    % Dimensions of the setup
    dims = get_dimensions(arm_model);

    % Connect to Tracking Tools
    if ~exist('natnet_client', 'var')
        [natnet_client, current_frame] = connect_tt();
    end

    % Connect to Controller Computer
    if ~exist('curvature_controller', 'var')
        curvature_controller = connect_cc();
    end

    % Simulation properties
    ShapeNumber = 2;
    ShapeTime = 45;
    Offset_x = 0.0381;
    Offset_y = 0.08;
    [Shape, InitalAngle] = getShape(ShapeNumber, ShapeTime, Offset_x, Offset_y); % Shape is the desired shape profile: rows = time pts along path, cols = time, x, y, theta desired
    time_col = 1;               % column corresponding to time in Shape
    x_col = 2;                  % column corresponding to x in Shape
    y_col = 3;                  % column corresponding to y in Shape
    theta_col = 4;              % column corresponding to theta in Shape
    n_current = 1;              % current index along the Shape
    shape_size = size(Shape,1);          % total number of points on the shape profile
    N = dims.S;

    k_min = -20;                % minum allowable curvature
    k_max = 20;                 % maximum allowable curvature

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = k_min*ones(N,1);
    ub = k_max*ones(N,1);
    nonlcon_empty = [];
    init = 1;

    T = 60;                     % total experiment time
    max_iterations = 3;         % number of iterations for nonlinear optimization solver
    k_previous = ones(N,1);     % used for low-pass filtering the set-change in reference curvatures
    k_command = zeros(N,1);
    %% Run the tracking
    % Start the counter
    t0 = tic;

    % Set first frame starting properties
    spos = dims.spos;
    prev_markers = NaN;
    prev_timestamp = NaN;

    % Prepare the ShapeHistory object
    shape_history = ShapeHistory(dims.S, T  * 100);

    % Move to start of path
    ShapeInit = [Shape(1,x_col:y_col), InitalAngle];
    options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3, 'MaxFunEvals',2000, 'TolX', 1e-6,'GradObj','on', 'GradConstr', 'off');
    k_init = 0.01*ones(N,1);
    k_target = fmincon(@simple_cost,k_init,A,b,Aeq,beq,lb,ub,@initnoncon,options);

    PlotChain(k_target, dims.srot, dims.lengths, N);
    plot(Shape(n_current,x_col), Shape(n_current,y_col),'ob')

    % While tracking is enabled
    while toc(t0) < T

            % Wait for next frame
            while current_frame.fLatency <= prev_timestamp
                % Busy wait
            end

            % Gather markers in the frame
            markers = NaN;
            while isnan(markers)
                % Get markers
                [markers, timestamp] = get_markers(current_frame, ...
                    dims.fly_zone, dims.vrange);
            end 

            % Calculate segments
            [ret_shape, ret_spos, ret_markers, error] = ...
               get_segments(markers, spos, dims.srot, ...
                dims.lengths, prev_markers);

            % If there was a problem with arclengths, try the straight segments assumption method
            if strcmp(error,'bad_arclength')
                [ret_shape, ret_spos, ret_markers, error] = ...
                    get_segments(markers, spos, dims.srot, ...
                    dims.lengths, NaN);
            end

            % If a result was returned, save it
            if isnan(error)
                % measure and update the history of the shape
                measured_curvature = ret_shape(5,:);
                shape_history.add(timestamp, ret_spos, ret_shape, k_target', k_command');

                % find Shape index corresponding to the current time
                n = find(Shape(:,time_col)>= toc(t0), 1, 'first');   

                % if the arm's target has advanced (it will always enter here first loop since n=1 and n_current = 0
                if( n ~= n_current)
                        %% %%%%%%%%%%%%%%%%%%%%%% Optimization IK Algorithm %%%%%%%%%%%
                        % initialize or re-initialize parameters
                        l = zeros(1,shape_size);             % distance each point on the path is from the tip
                        I = zeros(1,shape_size);             % arm segment on which each projected point falls
                        s = zeros(1,shape_size);             % the length from the beginning of segment I that thee projected point falls
                        w_desired = zeros(shape_size,2);     % desired point
                        w_measured = zeros(shape_size,2);    % measured location of projected point

                        % get current/measured curvatures
                        L = ret_shape(6,(1:6));

                        % the manipulator's tip is placed at the current, or nth, point on the Shape.
                        l(n) = 0.0;                    
                        I(n) = N;
                        s(n) = L(N); 

                        % Determine the index and length along the arm that each Shape point lies. 
                        for i = (n-1):-1:1
                            for j=i:(n-1)
                                temp = sqrt( (Shape(j+1,x_col)-Shape(j,x_col))^2 + (Shape(j+1,y_col)-Shape(j,y_col))^2 );
                                l(i) = l(i) + temp;
                            end

                            if( l(i) < L(N) )
                                I(i) = N;
                                s(i) = L(N)-l(i);
                            elseif( L(N) <= l(i) < sum(L(N-1:N)) )
                                I(i) = N-1;
                                s(i) = sum(L(N-1:N)) - l(i);
                            elseif( sum(L(N-1:N)) <= l(i) < sum(L(N-2:N)) )
                                I(i) = N-2;
                                s(i) = sum(L(N-2:N)) - l(i);
                            elseif( sum(L(N-2:N)) <= l(i) < sum(L(N-3:N)) )
                                I(i) = N-3;
                                s(i) = sum(L(N-3:N)) - l(i);
                            elseif( sum(L(N-3:N)) <= l(i) < sum(L(N-4:N)) )
                                I(i) = N-4;
                                s(i) = sum(L(N-4:N)) - l(i);
                            else
                                I(i) = N-5;
                                s(i) = sum(L(N-5:N)) - l(i);
                            end
                        end

                        % use nonlinear optimization solve to find curvatures subject to constraints, IK
                        options = optimoptions(@fmincon,'Algorithm', 'trust-region-reflective', 'TolCon', 0.005, 'MaxFunEvals', 2000, 'TolX', 1e-6,'GradObj','on', 'GradConstr', 'on');
                        
                        k_seed = k_target;
                        
                        before_time = toc(t0);
                        for temp = 1:max_iterations
                            [k_target, value] = fmincon(@complex_cost, k_seed,A,b,Aeq,beq,lb,ub, nonlcon_empty, options);
                            k_seed = k_target + 2*rand([N 1]);
                            for j=1:N
                                if k_seed(j) >= k_max
                                    k_seed(j) = k_max;
                                end
                                if k_seed(j) <= k_min
                                    k_seed(j) = k_min;
                                end
                            end
                        end
                        delta_time = toc(t0) - before_time
                        PlotChain(k_target, dims.srot, dims.lengths, N);
                        plot(Shape(n,x_col), Shape(n,y_col),'ob')
                        
                        n_current = n;
                end

                %Send filtered curvature setpoint to controllers
                alpha = 0.01;
                k_command = alpha*k_target + (1-alpha)*k_previous;
                k_previous = k_command;
                error = [k_command' - measured_curvature]'
                send_curvature_errors(curvature_controller, k_command', measured_curvature);

                % Prepare properties for the next frame
                spos = ret_spos;
                prev_markers = ret_markers;
                prev_timestamp = timestamp;
            else
                % Prepare properties for the next frame
                prev_timestamp = timestamp;

                % Display error message
                fprintf('NaN in a frame: %s.\n', error);
            end
    end

    % Save the shape history
    filename = sprintf('Data\\Optimization_IK_trials\\%s.mat', datestr(now));
    filename = strrep(filename,':','_');
    save(filename, 'shape_history');


    % Close the serial port communication
    disconnect_cc(curvature_controller)
    clear curvature_controller

    % Close tracking tools
    disconnect_tt(natnet_client)
    clear natnet_client

    function [cost, gcost] = complex_cost(k)
            ceq = zeros(1,n);        % nonlinear equalitity constraints
            ceq_hat = 0.0;        % nonlinear equalitity constraints
            gradceq = zeros(size(k,1), n);
            delta_k = 0.1; 

            for m=1:n
                w_desired(m,:) = Shape(m,x_col:y_col);                                              % the desired point on the shape
                [X, Y, Theta] = recursive_forward_kinematics(k, dims.srot, L, N, I(m), s(m));         % use FK to get  measured point on the shape
                w_measured(m,:) = [X, Y];                                                          
                ceq(m) = norm(w_desired(m,:)-w_measured(m,:), 'fro');                                    % the error between desired ans measured should be 0
                if m==1
                    ceq(m) = 2*ceq(m);
                end

                for( i=1:size(k,1) )
                    k_mod = k;
                    k_mod(i) = k_mod(i) + delta_k;
                    [X_hat, Y_hat, Theta_hat] = recursive_forward_kinematics(k_mod, dims.srot, L, N, I(m), s(m));
                    ceq_hat = norm(w_desired(m,:)-[X_hat, Y_hat], 'fro');
                    if m==1
                        ceq_hat = 2*ceq_hat;
                    end
                    gradceq(i, m) = (ceq_hat-ceq(m))/delta_k;
                end

            end

            cost = sum(ceq);
            gcost = sum(gradceq, 2);
    end

    function [c,ceq] = initnoncon(k)
        c = [];                  % nonlinear inequality constraints
        ceq = zeros(1,3);        % nonlinear equalitity constraints
        [X, Y, Theta] = recursive_forward_kinematics(k, dims.srot, dims.lengths, N, N, dims.lengths(N));
        ceq(1) = X - ShapeInit(1);
        ceq(2) = Y - ShapeInit(2);
        ceq(3) = Theta - ShapeInit(3);
    end

    function [E_tot, g] = simple_cost(k)
        E_tot = sum(k.^2);
        g = 2.*k;
    end
    
end
    
