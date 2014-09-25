%pid for polulu
% original PID: 20, 0.01, 0.01

% Ku = 36;
% Tu = 0.140;
% Kp = 0.7*Ku
% Ki = 0.4*Kp/Tu
% Kd = 0.15*Kp*Tu

%Curvature controller
Ku = 100;
Tu = 0.5;
Kp = 0.33*Ku
Ki = 2*Kp/Tu
Kd = Kp*Tu/3

