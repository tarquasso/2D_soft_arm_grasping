%% Simulation settings
close all
%clear all
clc

%% Add folders to the path
addpath('tracker/data_structures', ...
        'tracker/geometry', ...
        'tracker/kinematics', ...
        'tracker/peripherals/curvature_controller',  ...
        'tracker/peripherals/tracking_tools', ...
        'tracker/setup', ...
        'tracker/various')
%% Connect to controller computer
if ~exist('curvature_controller', 'var')
    curvature_controller = connect_cc();
end

%% Load experimental data
load('C:\Users\manipulator\Arm\Data\IKtrials\26-Sep-2013 13_16_47.mat')


%%

% Start the counter
T = 45; % path tracking
t0 = tic;

while toc(t0) < T
     %find the most recent command
     index = 1;
     
     while((shape_history.timestamps(index)-shape_history.timestamps(1)) < toc(t0))
        index = index + 1;
        if(index > shape_history.i)
            index = shape_history.i;
        end
     end
     
     cmd = shape_history.cmds(index,:);
     
     % Send the command
     fwrite(curvature_controller, cmd, 'int8');
     
     % wait
     pause(0.001);
end

%% Close the serial port communication
disconnect_cc(curvature_controller)
clear curvature_controller