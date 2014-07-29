clc
clear all

%load('C:\Users\sandwich\Desktop\softarm\cmd0_p2045.mat')

global positionData
global positionTime
global frameOfData %for debugging made global
global angles
%BYTES_PER_VOLT_2 = 1000.0/2.5;

[theClient, ls] = StartNatNetMatlab();
%[s, lh] = StartNIMatlab();
%arm = StartArm();

%span = getTimeSpan(cmdtraj);
%actuationTime = span(2) + 0.2337;
%experimentTime = actuationTime + 2;

%initialFrameTime = 0.0;
%experimentData = zeros(1,17);
%i = 1;

%seg2 = 2;
%seg3 = 3;
%seg4 = 4;

%value2 = 0;
%value3 = 0;
%value4 = 0;

% for i=1:10
%     pause(1);
%     positionData
%     positionTime
% end

%initialFrameTime = positionTime;

% tic
% while(toc <= experimentTime)
%     t = toc;
%     matlabTime = t;
%     currentPositionTime = positionTime - initialFrameTime;
%     currentPosition = positionData; 
%     currentInput = [value2, value3, value4];
%     experimentData(i,:) = [matlabTime, currentPositionTime, currentPosition, currentInput];
% 
%     if(toc <= actuationTime)
%         
%         u = eval(cmdtraj, t);
%         value2 = -u(1); 
%         value3 = -u(2); 
%         value4 = -u(3);
%     else
%         value2 = 0;
%         value3 = 0;
%         value4 = 0;
%     end
%     
%     setSegment(arm, seg2, value2);
%     setSegment(arm, seg3, value3);
%     setSegment(arm, seg4, value4);
%     arm.updateSegments();
% 
%     i = i + 1; 
%     pause(0.005);
% end
% 
% filename = sprintf('data/%s.mat', datestr(now));
% filename = strrep(filename,':','_');
% save(filename, 'experimentData');

% EndNatNetMatlab( theClient, ls );
