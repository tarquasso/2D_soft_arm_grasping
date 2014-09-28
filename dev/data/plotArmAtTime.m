function plotArmAtTime( currData, time, graspIndex )

gripperOnCurvature = 50;
gripperOffCurvature = 0.01;

% find index corresponding to time
timeIndex = find(currData.History.timestamps >= time, 1, 'first' );
if( isempty(timeIndex) )
    return;
end

% measured data corresponding to current time
l_kMeas = currData.History.kMeas(timeIndex, :);
l_arcLenMeas = currData.History.arcLenMeas(timeIndex, :);
l_objectX = currData.History.objectPosition(timeIndex,1);
l_objectY = currData.History.objectPosition(timeIndex,2);
l_kTargetGripper = currData.History.gripperkTarget(timeIndex,1);

%create a dummy arm
dummyArm = Arm2D(ExpTypes.Simulation);
dummyArm.setMeasuredCurvatures(l_kMeas);

if(l_kTargetGripper == gripperOnCurvature)
    l_gripperLengthShow = 0.130;
    l_gripperCurvatureShow = 35;
else
    l_gripperLengthShow = dummyArm.gripper2D.dims.length;
    l_gripperCurvatureShow = gripperOffCurvature;
end

if(timeIndex >= graspIndex)
    armColor = '-m';
    gripperColor = '-m';
    objectColor = 'm';
else
    armColor = '-c';
    gripperColor = '-c';
    objectColor = 'c';
end

dummyArm.gripper2D.setMeasuredCurvatures(l_gripperCurvatureShow);
dummyArm.gripper2D.setMeasuredLengths(l_gripperLengthShow);
dummyArm.setMeasuredLengths(l_arcLenMeas);

%create a dummy object
dummyObject = RoundObject();
dummyObject.setMeasuredState(l_objectX, l_objectY);

%plot the arm and object in measured pose
dummyArm.plotArmMeasToHandle(dummyArm.kMeas, armColor, gripperColor);
dummyObject.plotObjectToHandle(objectColor);


%delete dummy objects
dummyArm.delete();
dummyObject.delete();

end

