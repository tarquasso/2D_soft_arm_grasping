function [ ] = disconnect_cc( s )
%DISCONNECT_CC Disconnects from the curvature controller
%   The embedded curvature controller is actually implemented as a SIMULINK
%   simulation running on another Windows PC machine. The communication
%   with the controller is establisher via a serial link with the below
%   properties.

% Close the serial port communication
fclose(s);
delete(s);

end
