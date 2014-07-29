function [] = WaypointAndEnvelope()
    %% Simulation settings
    close all
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
    
    % Set first frame starting properties
    spos = dims.spos;
    prev_markers = NaN;
    prev_timestamp = NaN;
    
    % Connect to Tracking Tools
    if ~exist('natnet_client', 'var')
        [natnet_client, current_frame] = connect_tt();
    end

    % Connect to Controller Computer
    if ~exist('curvature_controller', 'var')
        curvature_controller = connect_cc();
    end

    %% initialize required paramters

    path_number = 3;
    time_along_path = 45;
    T = 60*2;
    offset_x = 0.0381;
    offset_y = 0.08;
    temp = getShape(path_number, time_along_path, offset_x, offset_y);
    path = [temp(:,2:end)];
    path_time = [temp(:,1); inf];
    x_col = 1;                  % column corresponding to x in Shape
    x_row = 1;
    y_col = 2;                  % column corresponding to y in Shape
    y_row = 2;
    theta_col = 3;              % column corresponding to theta in Shape
    P = size(path,1);          % total number of points on the shape profile
    alpha = 0.010;

    theta0 = dims.srot;              % is the current/measured initial orientation of the first segment
    N = dims.S;                      % is the total number of arm segments
    L = dims.lengths;        % is the current/measured length vector
    k_min = -20;                % minum allowable curvature
    k_max = 20;                 % maximum allowable curvature
    k_init = ones(N,1);          % initial measured  arm curvatures
    k_target = [];
    k_command = zeros(N,1);
    k_previous = ones(N,1);

    A = [];
    b = [];
    Aeq = [];
    beq = [];
    lb = k_min*ones(N,1);
    ub = k_max*ones(N,1);
    
    left_border_points = [];
    right_border_points = [];
    half_thickness = 35/1000/2;
    
    % Prepare the ShapeHistory object
    shape_history = ShapeHistory(dims.S, T  * 100);
    
    %% Generate environment borders
    [border] = getEnvironment();

    %% initialize plots, plot initial arm pose and initial path
    left_envelope_handle = plot([], []);
    right_envelope_handle = plot([], []);
    waypoint_handle = plot([], []);
    left_border_points_handle = plot([], []);
    right_border_points_handle = plot([], []);
    left_envelope_points_handle = plot([], []);
    right_envelope_points_handle = plot([], []);
    chain_handle = PlotChain(k_init, theta0, L, N);
    pause(1.0);
    
    %% set target as the first waypoint
    path_index = 1;
    waypoint = path(path_index,x_col:theta_col);
    options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon',2e-3, 'MaxFunEvals',2000, 'TolX', 1e-3,'GradObj','on', 'GradConstr', 'off');
    k_target = fmincon(@cost,k_init,A,b,Aeq,beq,lb,ub,@noncon,options);
    time_constant = [0.1, 0.01, 0.01, 0.01, 0.01, 0.01];
    
    delete(chain_handle);
    chain_handle = PlotChain(k_target, theta0, L, N);
    pause(1.0);
    
    % Start the counter
    t0 = tic;
    
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
            L = ret_shape(6,:);
            theta0 = ret_shape(1,1);
       
            shape_history.add(timestamp, ret_spos, ret_shape, k_target', k_command');
        
                if (path_time(path_index) <= toc(t0))
                    waypoint = path(path_index,x_col:theta_col);
                    delete(waypoint_handle);
                    waypoint_handle = plot(waypoint(x_col), waypoint(y_col), '.m', 'MarkerSize', 15);
                    waypoint_orthogonal =  [cos(waypoint(theta_col)+pi/2), sin(waypoint(theta_col)+pi/2)];
                    [border_indices]  = dsearchn(border',[waypoint(x_col:y_col)+alpha*waypoint_orthogonal; waypoint(x_col:y_col)-alpha*waypoint_orthogonal]);

                    left_border_points = [left_border_points; [border(x_row,border_indices(1)), border(y_row,border_indices(1))] ];
                    right_border_points = [right_border_points; [border(x_row,border_indices(2)), border(y_row,border_indices(2))] ];

                    plot(left_border_points(:,x_col), left_border_points(:,y_col), '.c', 'MarkerSize', 15);
                    plot(right_border_points(:,x_col), right_border_points(:,y_col), '.b', 'MarkerSize', 15);

                    options = optimoptions(@fmincon,'Algorithm', 'sqp', 'TolCon', 0.005, 'MaxFunEvals', 2000, 'TolX', 1e-4,'GradObj','off', 'GradConstr', 'off');
                    time_constant = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01];

                    before_time = toc(t0);
                    k_target = fmincon(@complex_cost,measured_curvature',A,b,Aeq,beq,lb,ub, @noncon, options)
                    delta_time = toc(t0) - before_time

                    [envelope_x, envelope_y] = getEnvelope(k_target, L, N, theta0, half_thickness);
                    center_index = (size(envelope_x,1)-1)/2;
                    left_envelope_x = envelope_x(1:center_index);
                    right_envelope_x = envelope_x(center_index+1:end);

                    left_envelope_y = envelope_y(1:center_index);
                    right_envelope_y = envelope_y(center_index+1:end);

                    [left_envelope_indices, left_envelope_to_border_distances] = dsearchn([left_envelope_x, left_envelope_y], left_border_points);
                    [right_envelope_indices, right_envelope_to_border_distances] = dsearchn([right_envelope_x, right_envelope_y], right_border_points);

                    delete(left_envelope_points_handle);
                    delete(right_envelope_points_handle);
                    delete(left_envelope_handle);
                    delete(right_envelope_handle);
                    delete(chain_handle);

                    chain_handle = PlotChain(k_target, theta0, L, N);
                    left_envelope_handle = plot(left_envelope_x, left_envelope_y, 'c');
                    right_envelope_handle = plot(right_envelope_x, right_envelope_y, 'b');
                    left_envelope_points_handle = plot(left_envelope_x(left_envelope_indices), left_envelope_y(left_envelope_indices),'.c', 'MarkerSize', 15 );
                    right_envelope_points_handle = plot(right_envelope_x(right_envelope_indices), right_envelope_y(right_envelope_indices),'.b', 'MarkerSize', 15 ); 
                    drawnow;
                    
                    path_index = path_index + 1;
                 end
                
            %Send filtered curvature setpoint to controllers
            time_matrix = diag(time_constant);
            k_command = time_matrix*k_target + (eye(N)-time_matrix)*k_previous;
            k_previous = k_command;
            e = [k_command' - measured_curvature]'
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

    
    function [cost] = complex_cost(k)

        [envelope_x, envelope_y] = getEnvelope(k, L, N, theta0, half_thickness);
        center_index = (size(envelope_x,1)-1)/2;
        left_envelope_x = envelope_x(1:center_index);
        right_envelope_x = envelope_x(center_index+1:end);
        
        left_envelope_y = envelope_y(1:center_index);
        right_envelope_y = envelope_y(center_index+1:end);
        
        [left_envelope_indices, left_envelope_to_border_distances] = dsearchn([left_envelope_x, left_envelope_y], left_border_points);
        [right_envelope_indices, right_envelope_to_border_distances] = dsearchn([right_envelope_x, right_envelope_y], right_border_points);
        
        [expanded_envelope_x, expanded_envelope_y] = getEnvelope(k, L, N, theta0, 3*half_thickness);
        expanded_left_envelope_x = expanded_envelope_x(1:center_index);
        expanded_right_envelope_x = expanded_envelope_x(center_index+1:end);
        expanded_left_envelope_y = expanded_envelope_y(1:center_index);
        expanded_right_envelope_y = expanded_envelope_y(center_index+1:end);
        
        for i = 1:size(left_envelope_indices,1)
           if inpolygon(left_border_points(i,x_col), left_border_points(i,y_col), [left_envelope_x; expanded_right_envelope_x], [left_envelope_y; expanded_right_envelope_y])
               left_envelope_to_border_distances(i) = -3*left_envelope_to_border_distances(i);
           end
        end
        
       for i = 1:size(right_envelope_indices,1)
           if inpolygon(right_border_points(i,x_col), right_border_points(i,y_col), [expanded_left_envelope_x; right_envelope_x], [expanded_left_envelope_y; right_envelope_y])
               right_envelope_to_border_distances(i) = -3*right_envelope_to_border_distances(i);
           end
        end
        
        cost = -sum(left_envelope_to_border_distances);
        cost = cost + -sum(right_envelope_to_border_distances);
    end
    
     
    function [c,ceq] = noncon(k)
        c = [];                  % nonlinear inequality constraints
        ceq = zeros(1,3);        % nonlinear equalitity constraints
        [target_x, target_y, target_theta] = recursive_forward_kinematics(k, theta0, L, N, N, L(N));
        ceq(1) = target_x - waypoint(1);
        ceq(2) = target_y - waypoint(2);
        ceq(3) = target_theta - waypoint(3);
    end

    function [E_tot, g] = cost(k)
        E_tot = sum(k.^2);
        g = 2.*k;
    end
    
end





