function plotArmAtGrasp( currData )

%find time index corresponding to when gripper target changes
gripperOn = 50;
gripperCurvatureShow = 35;
gripperLengthShow = 0.13;

graspIndex = find(currData.History.gripperkTarget >= gripperOn, 1, 'first' );

if( ~isempty(graspIndex) )
    graspSettleTime = 1.5;
    graspIndex = find(currData.History.timestamps >= ...
        currData.History.timestamps(graspIndex,1)+graspSettleTime, 1, 'first' );
else
    return;
end

l_kMeas = currData.History.kMeas(graspIndex, :);
l_arcLenMeas = currData.History.arcLenMeas(graspIndex, :);
l_objectX = currData.History.objectPosition(graspIndex,1);
l_objectY = currData.History.objectPosition(graspIndex,2);

%create a dummy arm
dummyArm = Arm2D(ExpTypes.Simulation);
dummyArm.setMeasuredCurvatures(l_kMeas);
dummyArm.gripper2D.setMeasuredCurvatures(gripperCurvatureShow);
dummyArm.gripper2D.setMeasuredLengths(gripperLengthShow);
dummyArm.setMeasuredLengths(l_arcLenMeas);

%create a dummy object
dummyObject = RoundObject();
dummyObject.setMeasuredState(l_objectX, l_objectY);

%plot the arm in measured grasping pose
dummyArm.plotArmMeasToHandle(dummyArm.kMeas);
dummyObject.plotObjectToHandle();

dummyArm.delete();
dummyObject.delete();

end

