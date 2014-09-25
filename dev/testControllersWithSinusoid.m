

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%
xMax = -0.10;
xMin = -0.20;
yMax = 0.30;
yMin = 0.20;
expTime = 18.0;

% figure;
% hold on;
% plot([0,0],[-1,1], '-k');
% plot([-1,1],[0,0], '-k');
% 
% plot([xMin,xMin],[yMin,yMax], '-k');
% plot([xMax,xMax],[yMin,yMax], '-k');
% plot([xMin,xMax],[yMin,yMin], '-k');
% plot([xMin,xMax],[yMax,yMax], '-k');
% 
ac = ArmController2D(ExpTypes.Tuning);
ac.start;

%pause(expTime)

l_kMeas = [];
l_kTarget = [];
l_time = [];
h = [];
segment = 1;
step = 0;
name = 'newContr1-1';
figure;
axis([0 expTime ac.arm2D.dims.kMin(segment)*1.4 ac.arm2D.dims.kMax(segment)*1.4]) ;
hold on;
plot([0, expTime], [0, 0]);

kMeasInit = ac.arm2D.kMeas;
   

tic;
while( toc <= expTime )
   
   pause(0.01);
   targets = ac.arm2D.kMeas;
   if(step==1)
   targets(segment) = double(2.0);%*sin(1/2*toc)+kMeasInit(segment);
   else
   targets(segment) = 2.0+double(1.0)*sin(1/2*toc);
   end
   %targets = zeros(1,6);
   ac.arm2D.setTargetCurvatures(targets);
   ac.arm2D.gripper2D.setTargetCurvatures(0); 
   ac.arm2D.actuate();
   
   l_kMeas = [l_kMeas,  ac.arm2D.kMeas(segment) ];
   l_kTarget = [l_kTarget,  ac.arm2D.kTarget(segment) ];
   l_time = [l_time, toc];
   
   delete(h);
   h = plot(l_time, l_kMeas, 'r', l_time, l_kTarget, 'b', 'LineWidth', 2);
   drawnow;
end

   targets = ac.arm2D.kMeas;
   ac.arm2D.setTargetCurvatures(targets);
   ac.arm2D.gripper2D.setTargetCurvatures(0); 
   ac.arm2D.actuate();
ac.stop;
ac.delete;
% currpath = pwd;
%    cd ..;
%    cd exp;
%    cd singlesegments;
% if(step ==1)
%    saveas(gcf,['step_seg',num2str(segment),'-',name,'-eps'],'epsc');
% else
%    saveas(gcf,['sinus_seg',num2str(segment),'-',name,'-eps'],'epsc');
% end
% cd(currpath);
