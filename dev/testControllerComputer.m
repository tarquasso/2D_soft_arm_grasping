

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
xMax = -0.10;
xMin = -0.20;
yMax = 0.30;
yMin = 0.20;
expTime = 45.0;

figure;
hold on;
plot([0,0],[-1,1], '-k');
plot([-1,1],[0,0], '-k');

plot([xMin,xMin],[yMin,yMax], '-k');
plot([xMax,xMax],[yMin,yMax], '-k');
plot([xMin,xMax],[yMin,yMin], '-k');
plot([xMin,xMax],[yMax,yMax], '-k');

ac = ArmController2D;
ac.start;

pause(expTime)

% l_kMeas = [];
% l_kTarget = [];
% l_time = [];
% h = [];
% segment = 6;
% 
% figure;
% axis([0 expTime ac.arm2D.dims.kMin(segment) ac.arm2D.dims.kMax(segment)]) ;
% hold on;
% plot([0, expTime], [0, 0]);
% 
% tic;
% while( toc <= expTime )
%    toc
%    pause(0.010);
%    targets = ac.arm2D.kMeas;
%    targets(segment) = -10.0*sin(toc);
%    ac.arm2D.setTargetCurvatures([targets]);
%    ac.arm2D.gripper2D.setTargetCurvatures(0); 
%    ac.arm2D.actuate();
%    
%    l_kMeas = [l_kMeas,  ac.arm2D.kMeas(segment) ];
%    l_kTarget = [l_kTarget,  ac.arm2D.kTarget(segment) ];
%    l_time = [l_time, toc];
%    
%    delete(h);
%    h = plot(l_time, l_kMeas, 'r', l_time, l_kTarget, 'b', 'LineWidth', 2);
%    drawnow;
% end

ac.stop;
ac.delete;