%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
xMax = -0.10;
xMin = -0.25;
yMax = 0.35;
yMin = 0.12;
expTime = 10;

figure;
hold on;
plot([0,0],[-1,1], '-k');
plot([-1,1],[0,0], '-k');

plot([xMin,xMin],[yMin,yMax], '-k');
plot([xMax,xMax],[yMin,yMax], '-k');
plot([xMin,xMax],[yMin,yMin], '-k');
plot([xMin,xMax],[yMax,yMax], '-k');

ac = ArmController2D(ExpTypes.PhysicalExperiment);

ac.start;

%timeoutDlg(@input, expTime, 'Enter 1 to cancel:');
pause(expTime);

delete(ac);

