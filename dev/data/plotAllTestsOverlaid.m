clear all
clc

% Find object files
addpath ../ % add dev folder for class definitions

%% setup figure
h = figure;

axes1 = axes('Parent',h,'YTick',[0 0.1 0.2 0.3 0.4 0.5 0.6],...
    'XTick',[-0.4 -0.3 -0.2 -0.1 0 0.1 0.2],...
    'PlotBoxAspectRatio',[1 1 1],...
    'LineWidth',1.5,...
    'FontSize',14);

xlim(axes1,[-0.35 0.2]);
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
      
% plot([xMin,xMin],[yMin,yMax], '-r', 'LineWidth', 2.0);
% plot([xMax,xMax],[yMin,yMax], '-r', 'LineWidth', 2.0);
% plot([xMin,xMax],[yMin,yMin], '-r', 'LineWidth', 2.0);
% plot([xMin,xMax],[yMax,yMax], '-r', 'LineWidth', 2.0);

%% plot experiments for pos 1 c
addpath pos1c % first experiment folder

currData = load('pos1c\27-Sep-2014 00_16_24.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos1c\27-Sep-2014 00_17_59.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos1c\27-Sep-2014 00_36_13.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos1c\27-Sep-2014 00_37_49.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos1c\27-Sep-2014 00_39_26.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );
%% plot experiments for pos 2 a
addpath pos2a % first experiment folder

currData = load('pos2a\27-Sep-2014 01_22_12.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos2a\27-Sep-2014 01_31_30.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos2a\27-Sep-2014 01_33_06.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos2a\27-Sep-2014 01_34_47.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos2a\27-Sep-2014 01_36_37.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

%% plot experiments for pos 3 a
addpath pos3a % first experiment folder

currData = load('pos3a\27-Sep-2014 01_49_33.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos3a\27-Sep-2014 01_58_10.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos3a\27-Sep-2014 02_03_32.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos3a\27-Sep-2014 02_06_20.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos3a\27-Sep-2014 02_08_58.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );
%% plot experiments for pos 4 a
addpath pos4a % first experiment folder

currData = load('pos4a\27-Sep-2014 03_22_34.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos4a\27-Sep-2014 03_24_19.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos4a\27-Sep-2014 03_26_43.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos4a\27-Sep-2014 03_28_38.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos4a\27-Sep-2014 03_30_46.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );
%% plot experiments for pos 5 a
addpath pos5a % first experiment folder

currData = load('pos5a\27-Sep-2014 03_40_34.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos5a\27-Sep-2014 03_45_02.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos5a\27-Sep-2014 03_48_11.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos5a\27-Sep-2014 03_50_25.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

currData = load('pos5a\27-Sep-2014 03_52_14.mat');
plotArmAtGrasp( currData );
plotArmAtRelease( currData );

%% adjust figure once data is plotted









