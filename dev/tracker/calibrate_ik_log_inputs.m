%% Simulation settings
close all
%clear all
clc
% Model of the arm
arm_model = 'A';
segment = 1;


%% Input function properties - requested tip positions and rotations
% format [(relative_x + base_x), (relative_y + base_y)]

%use this for point-to-point movements
    tpos_req = [(0.13)+0.2672; (0.14)+0.1805];
    trot_req = pi*(0.5);
    decision_time = 100.0;

%use this for moving in a vertical line
%     x_start = -0.10;
%     y_start = 0.07;
%     theta_start = pi;
%     
%     x_transition = -0.10;
%     y_transition = 0.16;
%     theta_transition = pi/2;
%     
%     x_end = 0.00;
%     y_end = 0.16;
%     theta_end = pi/2;
%     
%     start_time = 15.0;
%     transition_time1 = 25.0;
%     transition_time2 = 30.0;
%     end_time = 40.0;
%     
%     decision_time = 100.0; % make large for line tracking

% Simulation properties
T = 45;
T_mes = [];
k_previous = ones(6,1);
position_error = [];
position_threshold = 0.0;
theta_threshold = 0.0;
thresholds_determined = 0;
k_current = zeros(6,1);
cmd = zeros(1,12);

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

if ~exist('curvature_controller', 'var')
    curvature_controller = connect_cc();
end

%% Set up the figure
TPOS_err = [];
error_figure = figure;
tpos_err_axes = subplot(2,1,1);
plot([0 T], [0 0], 'k')
hold on
tpos_err_series = plot(Inf,Inf,'r');
title('Tip position error')
xlabel('t (s)')
ylabel('distance (m)')

TROT_err = [];
rot_err_axes = subplot(2,1,2);
plot([0 T], [0 0], 'k')
hold on
trot_err_series = plot(Inf,Inf,'b');
xlabel('t (s)')
ylabel('rotation (rad)')
title('Tip rotation error')

drawnow

%% Run the tracking
% Start the counter
t0 = tic;

% Set first frame starting properties
spos = dims.spos;
prev_markers = NaN;
prev_timestamp = NaN;

% Prepare the ShapeHistory object
shape_history = ShapeHistory(dims.S, T * 100);

% While tracking is enabled
while toc(t0) < T
    
    % Wait for next frame
    while current_frame.fLatency <= prev_timestamp
        % Busy wait
    end
    
    % Gather markers in the frame
    markers = NaN;
    while isnan(markers)
        % Calculate current time
        t = toc(t0);
        % Get markers
        [markers, timestamp] = get_markers(current_frame, ...
            dims.fly_zone, dims.vrange);
    end
    
    % Calculate segments
    [ret_shape, ret_spos, ret_markers, error] = ...
       get_segments(markers, spos, dims.srot, ...
        dims.lengths, prev_markers);
    
    % If there was a problem with arclengths, try the straight segments
    % assumption method
    if strcmp(error,'bad_arclength')
        [ret_shape, ret_spos, ret_markers, error] = ...
            get_segments(markers, spos, dims.srot, ...
            dims.lengths, NaN);
    end
    
    % If a result was returned, save it
    if isnan(error)
        % Update the history of the shape
        % shape_history.add(timestamp, ret_spos, ret_shape, k_current);
                
                %%%%%%%%%%%%%%%%%%% use indented section for line movement %%%%%%%%%%%%%%%%%%%
                % determine requested position
        
%                 if( t <= start_time )
%                     x_tip_desired = x_start;
%                     y_tip_desired = y_start;
%                     theta_tip_desired = theta_start;
%                     %alpha = 0.1;
%                     alpha = 0.15;
%                 elseif((start_time < t) && (t <= transition_time1))
%                     x_tip_desired = x_start;
%                     y_tip_desired = y_start + (t-start_time)/(transition_time1-start_time)*(y_transition-y_start);
%                     theta_tip_desired = theta_start;
%                     %alpha = 0.1;
%                     alpha = 1.0;
%                 elseif((transition_time1 < t) && (t <= transition_time2))
%                     x_tip_desired = x_transition;
%                     y_tip_desired = y_transition;
%                     theta_tip_desired = theta_transition;
%                     %alpha = 0.1;
%                     alpha = 1.0;
%                 elseif((transition_time2 < t) && (t <= end_time))
%                     x_tip_desired = x_transition + (t-transition_time2)/(end_time-transition_time2)*(x_end-x_transition);
%                     y_tip_desired = y_end;
%                     theta_tip_desired = theta_transition;
%                     %alpha = 0.1;
%                     alpha = 1.0;
%                 else
%                     x_tip_desired = x_end;
%                     y_tip_desired = y_end;
%                     theta_tip_desired = theta_end;
%                     alpha = 1.0;
%                 end
%         
%                 tpos_req = [x_tip_desired + ret_spos(1); y_tip_desired + ret_spos(2)];
%                 trot_req = theta_tip_desired;
                
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % determine new errors
        tpos_err = p_norm(tpos_req, ret_shape(2:3,end), 2);
        trot_err = trot_req - ret_shape(4,end);
        
        % update error matrices
        T_mes = [T_mes t];
        TPOS_err = [TPOS_err tpos_err];
        TROT_err = [TROT_err trot_err];
        
        % Measure curvature
        measured = ret_shape(5,:);
        
        % Calculate the requested curvature
        tpos_req_rel = tpos_req - ret_spos;
        tip_measured_rel = [ret_shape(2:3,end) - ret_spos; ret_shape(4,end)];
        
        %% detemine required curvature update
        max_iterations = 300;
        k_current = transpose(measured);
        length_array = ret_shape(6,(1:6));
   
        if( decision_time <= t && thresholds_determined == 0 )
            s = movingstd(TPOS_err, 80, 'central');
            position_threshold = mean(s((end-10):end));
            s = movingstd(abs(TROT_err), 80, 'central');
            theta_threshold = mean(s((end-10):end));
            thresholds_determined = 1;
        end
        
        if( (TPOS_err(end) <= 2*position_threshold) &&  (abs(TROT_err(end)) <= 2*theta_threshold) )
             k_current = transpose(measured);
        else
            for i=0:max_iterations
            
                k_next = inverse_kinematics(tpos_req_rel(1), tpos_req_rel(2), ...
                trot_req, length_array, k_current, dims.srot);
            
                k_current = k_next;
                %k_current = [k_current(1); k_next];
            
            end
            
            alpha = 0.05;
            k_current = alpha*k_current + (1-alpha)*k_previous;
            k_previous = k_current
        
        end
        
        %% Send updated curvature setpoint to controllers

        cmd = send_curvature_errors(curvature_controller, k_current', measured);
        shape_history.add(timestamp, ret_spos, ret_shape, k_current, cmd);

        
        %% Update the error figure
        
        set(tpos_err_series,'xdata',T_mes,'ydata',TPOS_err)
        set(trot_err_series,'xdata',T_mes,'ydata',TROT_err)
        drawnow
        
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
filename = sprintf('Data\\IKtrials\\%s.mat', datestr(now));
filename = strrep(filename,':','_');
save(filename, 'shape_history');


% Close the serial port communication
disconnect_cc(curvature_controller)
clear curvature_controller

% Close tracking tools
disconnect_tt(natnet_client)
clear natnet_client