function plotArmPlan( currData )

% find initial index corresponding to time
time = 1.0;
timeIndex = find(currData.History.timestamps >= time, 1, 'first' );
if( isempty(timeIndex) )
    return;
end

% measured data corresponding to current time
l_kMeas = currData.History.kMeas(timeIndex, :);
l_arcLenMeas = currData.History.arcLenMeas(timeIndex, :);
l_objectX = currData.History.objectPosition(timeIndex,1);
l_objectY = currData.History.objectPosition(timeIndex,2);
%l_kTargetGripper = currData.History.gripperkTarget(timeIndex,1);

%create a dummy arm
dummyArm = Arm2D(ExpTypes.Simulation);
dummyArm.setMeasuredCurvatures(l_kMeas);
% dummyArm.gripper2D.setMeasuredCurvatures(l_gripperCurvatureShow);
% dummyArm.gripper2D.setMeasuredLengths(l_gripperLengthShow);
dummyArm.setMeasuredLengths(l_arcLenMeas);

%create a dummy object
dummyObject = RoundObject();
dummyObject.setMeasuredState(l_objectX, l_objectY);

%plot the optimal plan
gripperColor = '--k';
armColor = '--k';
circleColor = '--k';

%plot the optimal plan
gripperColor = '-k';
armColor = '-b';
circleColor = '-g';

for i = 1:currData.History.plannerResults.nMov
    l_kTarget = currData.History.plannerResults.kOptimal(i, :);
    dummyArm.plotArmTargetToHandle(l_kTarget, armColor, gripperColor);
    plot(currData.History.plannerResults.tipOptimal(i, 1), ...
        currData.History.plannerResults.tipOptimal(i, 2), '.k', 'MarkerSize', 20)
    l_radius = currData.History.plannerResults.allRadii(1, i);
    l_circleCentX = currData.History.objectPosition(timeIndex, 1);
    l_circleCentY = currData.History.objectPosition(timeIndex, 2);
    plotCircle(l_circleCentX, l_circleCentY, l_radius, circleColor)
end

%delete dummy objects
dummyArm.delete();
dummyObject.delete();

end

function plotCircle(x,y,r, circleColor)
    ang=0:0.01:2*pi; 
    xp=r*cos(ang);
    yp=r*sin(ang);
    plot(x+xp,y+yp, circleColor, 'LineWidth', 2.0);
end

