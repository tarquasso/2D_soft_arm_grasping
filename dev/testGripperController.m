

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Initialize Figure %%%%%%%%%%%%%%%%%%%%%%%%

ac = ArmController2D(ExpTypes.Tuning);
ac.start;
expTime = 10;
amplitude = 25;

l_kMeas = [];
l_kTarget = [];
l_time = [];
h = [];
step = 0;
name = 'newContr1-1';
figure;
axis([0 expTime ac.arm2D.gripper2D.dims.kMin*1.2 ac.arm2D.gripper2D.dims.kMax*1.9]) ;
hold on;
plot([0, expTime], [0, 0]);

kMeasInit = ac.arm2D.gripper2D.kMeas;


tic;
while( toc <= expTime )
    
    pause(0.01);
    targets = ac.arm2D.kMeas;
    if(step==1)
        targetGrip = double(amplitude);%*sin(1/2*toc)+kMeasInit(segment);
    else
        targetGrip = amplitude+double(amplitude)*sin(1/2*toc-pi/2);
    end
    %targets = zeros(1,6);
    ac.arm2D.setTargetCurvatures(targets);
    ac.arm2D.gripper2D.setTargetCurvatures(targetGrip);
    ac.arm2D.actuate();
    
    l_kMeas = [l_kMeas,  ac.arm2D.gripper2D.kMeas ];
    l_kTarget = [l_kTarget,  ac.arm2D.gripper2D.kTarget];
    l_time = [l_time, toc];
    
    delete(h);
    h = plot(l_time, l_kMeas, 'r', l_time, l_kTarget, 'b', 'LineWidth', 2);
    drawnow;
end

targets = ac.arm2D.kMeas;
ac.arm2D.setTargetCurvatures(targets);
ac.arm2D.gripper2D.setTargetCurvatures(0.01);
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
