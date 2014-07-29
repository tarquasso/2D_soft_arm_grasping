function [ s ] = connect_cc( )
%CONNECT_CC Connects to the embedded curvature controller
%   The embedded curvature controller is actually implemented as a SIMULINK
%   simulation running on another Windows PC machine. The communication
%   with the controller is establisher via a serial link with the below
%   properties.
%
%   The returned object 's' is a Serial Port Object.

% Establish the serial port communication
s = serial('COM1');
set(s,'BaudRate',57600,'DataBits',8,'StopBits',1,'Parity','none');
fopen(s);

end
