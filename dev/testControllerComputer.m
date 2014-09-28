%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
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

%% run experiment
expTime = 70;
ac = ArmController2D(ExpTypes.PhysicalExperiment);

ac.start;

%timeoutDlg(@input, expTime, 'Enter 1 to cancel:');
pause(expTime);
if(ac.isvalid ==1)
    delete(ac);
end
