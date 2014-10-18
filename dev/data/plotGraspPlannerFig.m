clc
clear all

%% Find experiment files
addpath ../ % add dev folder for class definitions

%% setup figure
h = figure;

axes1 = axes('Parent',h,'YTick',[0 0.1 0.2 0.3 0.4 0.5 0.6],...
    'XTick',[-0.5 -0.4 -0.3 -0.2 -0.1 0 0.1],...
    'PlotBoxAspectRatio',[1 1 1],...
    'LineWidth',1.5,...
    'FontSize',14);

xlim(axes1,[-0.4 0.15]);
ylim(axes1,[0.0 0.55]);
xlabel('x [m]', 'FontSize', 20);
ylabel('y [m]', 'FontSize', 20);

box(axes1,'on');
grid(axes1,'on');
hold(axes1,'all');

xMax = 0.05;
xMin = -0.1;
yMax = 0.42;
yMin = 0.35;

rectangle('Position',[xMin,yMin,(xMax-xMin),(yMax-yMin)],'Curvature',[0,0],...
          'FaceColor','r')

%% plot experiment for pos 5 a
addpath pos5a % first experiment folder
currData = load('pos5a\27-Sep-2014 03_50_25.mat');

% plot plan
plotArmPlan( currData );

%plot arm at time init
time = 1;
graspIndex = 500;
plotArmAtTime( currData, time, graspIndex )
