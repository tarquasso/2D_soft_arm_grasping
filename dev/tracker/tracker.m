function varargout = tracker(varargin)
% TRACKER MATLAB code for tracker.fig
%      TRACKER, by itself, creates a new TRACKER or raises the existing
%      singleton*.
%
%      H = TRACKER returns the handle to a new TRACKER or the handle to
%      the existing singleton*.
%
%      TRACKER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKER.M with the given input arguments.
%
%      TRACKER('Property','Value',...) creates a new TRACKER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before tracker_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to tracker_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help tracker

% Last Modified by GUIDE v2.5 17-Apr-2013 14:05:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @tracker_OpeningFcn, ...
                   'gui_OutputFcn',  @tracker_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before tracker is made visible.
function tracker_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to tracker (see VARARGIN)

% Choose default command line output for tracker
handles.output = hObject;

% Add folders
addpath('data_structures', ...
        'geometry', ...
        'kinematics', ...
        'peripherals/curvature_controller', ...
        'peripherals/tracking_tools', ...
        'setup', ...
        'various')

% Get all dimensions of the experimental setup
setappdata(hObject,'dims',get_dimensions('A'));
dims = getappdata(hObject,'dims');

% Set up the graph
hold on
axis(dims.axis);
xlabel('x (m)')
ylabel('y (m)')
handles.fly_zone = plot(handles.surface_axes, ...
    dims.fly_zone(1,:), dims.fly_zone(2,:),'k--');
handles.markers = scatter(Inf,Inf);
handles.mes_curve = plot(Inf,Inf,'b');
handles.req_curve = plot(Inf,Inf,'r--');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes tracker wait for user response (see UIRESUME)
% uiwait(handles.main_figure);


% --- Outputs from this function are returned to the command line.
function varargout = tracker_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_connect.
function pushbutton_connect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Connect to Tracking Tools
[natnet_client, current_frame] = connect_tt();
setappdata(handles.main_figure, 'natnet_client', natnet_client);
setappdata(handles.main_figure, 'current_frame', current_frame);

% Connect to curvature controller
setappdata(handles.main_figure, 'curvature_controller', connect_cc());

% Deactivate connection pushbutton
set(hObject,'enable','off');


% --- Executes on button press in togglebutton_track.
function togglebutton_track_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton_track (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton_track

% Return if the button was turned off
if ~get(hObject,'Value')
    return
end

% Get dimensions
dims = getappdata(handles.main_figure,'dims');
current_frame = getappdata(handles.main_figure,'current_frame');

% Set up the frames buffer
N = get(handles.buffer_edit,'Value');
frames = ShapeHistory(dims.S,N);

% Set first frame starting properties
prev_start_pt = dims.start_pt;
prev_markers = NaN;
prev_timestamp = NaN;

% Start the timer
t0 = tic;

% While tracking is enabled
while get(hObject,'Value')
    % Wait for next frame
    while current_frame.fLatency <= prev_timestamp
        % Busy wait
    end
    
    % Gather markers in the frame
    markers = NaN;
    while isnan(markers)
        [markers, timestamp] = get_markers(current_frame, ...
            dims.fly_zone, dims.vertical_range);
    end
    
    % Calculate segments
    [ret_shape, ret_start_pt, ret_markers, error] = ...
        calculate_segments(markers, prev_start_pt, dims.start_rot, ...
        dims.lengths, prev_markers);
    
    % If there was a problem with arclengths, try the straight segments
    % assumption method
    if strcmp(error,'bad_arclength')
        [ret_shape, ret_start_pt, ret_markers, error] = ...
            calculate_segments(markers, prev_start_pt, dims.start_rot, ...
            dims.lengths, NaN);
        if ~isnan(ret_shape)
            fprintf('Saved frame %d by falling back to default curvatures.\n', frames.i);
        end
    end
    
    % If a result was returned, save it
    if ~isnan(ret_shape)
        % Draw the position of the arm
        set(handles.markers,'xdata',ret_shape(:,1),'ydata',ret_shape(:,2))
        mes_curve = integrate_arm_shape(ret_start_pt, ret_shape);
        set(handles.mes_curve,'xdata',mes_curve(:,1),'ydata',mes_curve(:,2))
        drawnow
        
        % Save the properties
        frames.add(timestamp, ret_start_pt, ret_shape);
        
        % Show current curvature error
        tip_error_t = [tip_error_t toc(t0)];
        tip_error_sq = (x_tip_desired - ret_shape(end,1))^2 + ...
            (y_tip_desired - ret_shape(end,2))^2;
        tip_error_mag = [tip_error_mag tip_error_sq^0.5];
        set(tip_error_ref_plot, 'xdata', [0 tip_error_t(end)]);
        set(tip_error_plot, 'xdata', tip_error_t, 'ydata', tip_error_mag)
        
        % Calculate the required curvatures
        x_des = x_tip_desired - ret_start_pt(1);
        y_des = y_tip_desired - ret_start_pt(2);
        L = mean(ret_shape(:,3));
        k_current = ret_shape(:,4);
        theta0 = dims.start_rot;
        k_req = InverseKinematics(x_des, y_des, theta_tip_desired, ...
                                  L, ...
                                  k_current, ...
                                  theta0);
        %send_curvature_errors(curvature_controller, k_req, k_current);
        
        % Prepare properties for the next frame
        prev_start_pt = ret_start_pt;
        prev_markers = ret_markers;
        prev_timestamp = timestamp;
    else
        % Prepare properties for the next frame
        prev_timestamp = timestamp;
        
        % Display error message
        fprintf('NaN after frame %d: %s.\n', frames.i-1, error);
    end
end

% Close the curvature controller
%disconnect_curvature_controller(curvature_controller);
clear curvature_controller

% Save the frames in main figure appdata
setappdata(handles.main_figure, 'frames', frames)

disp('End of tracking')

function buffer_edit_Callback(hObject, eventdata, handles)
% hObject    handle to buffer_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of buffer_edit as text
%        str2double(get(hObject,'String')) returns contents of buffer_edit as a double


% --- Executes during object creation, after setting all properties.
function buffer_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to buffer_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit_tip_x_Callback(hObject, eventdata, handles)
% hObject    handle to edit_tip_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_tip_x as text
%        str2double(get(hObject,'String')) returns contents of edit_tip_x as a double


% --- Executes during object creation, after setting all properties.
function edit_tip_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_tip_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_tip_y_Callback(hObject, eventdata, handles)
% hObject    handle to edit_tip_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_tip_y as text
%        str2double(get(hObject,'String')) returns contents of edit_tip_y as a double


% --- Executes during object creation, after setting all properties.
function edit_tip_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_tip_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_tip_orient_Callback(hObject, eventdata, handles)
% hObject    handle to edit_tip_orient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_tip_orient as text
%        str2double(get(hObject,'String')) returns contents of edit_tip_orient as a double


% --- Executes during object creation, after setting all properties.
function edit_tip_orient_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_tip_orient (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_time_Callback(hObject, eventdata, handles)
% hObject    handle to edit_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_time as text
%        str2double(get(hObject,'String')) returns contents of edit_time as a double


% --- Executes during object creation, after setting all properties.
function edit_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_frame_Callback(hObject, eventdata, handles)
% hObject    handle to edit_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_frame as text
%        str2double(get(hObject,'String')) returns contents of edit_frame as a double


% --- Executes during object creation, after setting all properties.
function edit_frame_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_frame (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit_skipped_Callback(hObject, eventdata, handles)
% hObject    handle to edit_skipped (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_skipped as text
%        str2double(get(hObject,'String')) returns contents of edit_skipped as a double


% --- Executes during object creation, after setting all properties.
function edit_skipped_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_skipped (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function surface_axes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to surface_axes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate surface_axes


% --- Executes when user attempts to close main_figure.
function main_figure_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to main_figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% If handles is not a struct it means that it hasn't been initialized
% (tracker.fig was opened instead of tracker.m) and trying to access it
% will create an error. That's why need to check for it.

if isa(handles, 'struct')
    % Close the connection to Tracking Tools
    if isappdata(handles.main_figure, 'natnet_client')
        natnet_client = getappdata(handles.main_figure, 'natnet_client');
        disconnect_tt(natnet_client);
    end

    % Close the connection to the curvature controller
    if isappdata(handles.main_figure, 'curvature_controller')
        curvature_controller = getappdata(handles.main_figure, 'curvature_controller');
        disconnect_cc(curvature_controller);
    end
end

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on button press in tip_error_togglebutton.
function tip_error_togglebutton_Callback(hObject, eventdata, handles)
% hObject    handle to tip_error_togglebutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tip_error_togglebutton

% If the button is being pressed down, open the tip error figure

if get(hObject,'Value')
    % Set up the error figure
    figure;
    ref = plot([0 0], [0 0]);
    hold on
    error = plot(Inf, Inf, 'k');
    mag = [];
    t = [];
    xlabel('time (s)')
    ylabel('magnitude of tip error (m)')
    title('Tip error')
end
