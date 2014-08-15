

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
xMax = 0.15;
xMin = -0.15;
yMax = 0.35;
yMin = 0.30;
expTime = 20.0;

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

pause(20.0)

ac.stop;

% l_kMeas = [];
% l_kTarget = [];
% l_time = [];
% h = [];
% segment = 5;
% 
% figure;
% axis([0 expTime ac.arm2D.dims.kMin ac.arm2D.dims.kMax]) ;
% hold on;
% plot([0, expTime], [0, 0]);
% 
% tic;
% while( toc <= expTime )
%    toc
%    %pause(0.010);
%    %targets = ac.arm2D.kMeas';
%    targets = -5.0*ones(6,1); %*sin(toc);
%    ac.arm2D.setTargetCurvatures([targets]); 
%    
%    l_kMeas = [l_kMeas,  ac.arm2D.kMeas(segment) ];
%    l_kTarget = [l_kTarget,  ac.arm2D.kTarget(segment) ];
%    l_time = [l_time, toc];
%    
%    delete(h);
%    h = plot(l_time, l_kMeas, 'r', l_time, l_kTarget, 'b', 'LineWidth', 2);
% end

ac.delete;