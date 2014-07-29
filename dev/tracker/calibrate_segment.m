%% Simulation settings
% Model of the arm
arm_model = 'A';
segment = 2;

% 6 ignore
% 5 good
% 4 good
% 3 good 
% 2 good
% 1 good

% Input function properties
T = 18;
min_k = -20;
max_k = -10; 
period = T/2;

% Calculate input function
dt = 0.001;
T_req = 0:dt:T;
K_req = min_k + (sin(2*pi*T_req/period)+1)/2*(max_k-min_k);
T_mes = [];
K_mes = [];
K_err = [];
k_current = zeros(6,1);

%% Startup
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
control_figure = figure;
curvature_axes = subplot(3,1,[1 2]);
plot(T_req,K_req,'b--')
hold on
k_mes_series = plot(Inf,Inf,'r');
title(sprintf('Curvature of segment %d', segment))
legend('requested', 'measured')
% xlabel('t (s)')
ylabel('curvature (1/m)')

error_axes = subplot(3,1,3);
plot([0 T], [0 0], 'k')
hold on
k_err_series = plot(Inf,Inf,'b');
xlabel('t (s)')
ylabel('curvature (1/m)')
title('Curvature error')

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
        shape_history.add(timestamp, ret_spos, ret_shape, k_current);
        
        % Measure curvature
        index = round( min(t/T,1) * numel(T_req) );
        index = max(index,1);
        k_req = K_req(index);
        
        % Send the error
        measured = ret_shape(5,:);
        target = measured;
        %target(1) = 0;
        %target(2) = 0;
        %target(3) = -25;
        %target(4) = +25;
        %target(5) = k_req;
        %target(6) = k_req;
        target(segment) = k_req;
        
        send_curvature_errors(curvature_controller, target, measured)
        
        % Update the measured curvature
        K_mes = [K_mes measured(segment)];
        K_err = [K_err k_req-measured(segment)];
        T_mes = [T_mes t];
        set(k_mes_series,'xdata',T_mes,'ydata',K_mes)
        set(k_err_series,'xdata',T_mes,'ydata',K_err)
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
filename = sprintf('Data\\Segment calibration\\%s.mat', datestr(now));
filename = strrep(filename,':','_');
save(filename, 'shape_history');

% Close the serial port communication
disconnect_cc(curvature_controller)
clear curvature_controller

% Close tracking tools
disconnect_tt(natnet_client)
clear natnet_client