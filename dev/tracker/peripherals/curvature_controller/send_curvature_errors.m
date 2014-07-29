function command = send_curvature_errors( controller, target, measured )
%SEND_CURVATURE_ERRORS Sends error to the embedded PID curvature controller
%   INPUT:
%   controller	Serial Port Object - curvature controller object
%   target              1xS vector - target curvatures
%   measured            1xS vector - measured curvatures

function [ k ] = select_curvature(target, measured)
%   Selects the appropriate curvature control bits. If the measured
%   curvature is above a safe hard-coded threshold, we are safe in
%   actuating using the motor opposite to the current curvature.
%   
%   However if we are in a transition region close to 0 curvature, we are
%   not sure what the actual curvature of the segment is. To be safe, we
%   will actuate the segment with the motor opposite to the target
%   curvature.
    if abs(measured) >= threshold
        if measured >= 0.0
            k = 1;
        else
            k = -1; 
        end
    else
        if target >= 0.0
            k = 1; %1
        else
            k = -1; %-1
        end
    end
end

% Threshold and multiplier. The threshold corresponds to a curvature of
% 5 m^-1, or alternatively a radius of curvature of 20 cm. The theory is
% that curvature above the value of 5 will be noise-immune.
% 
% The multiplier c will give a curvature difference of 40 the maximum value
% of error at +127. Curvature of 40 m^-1 corresponds to a radius of
% curvature of 2.5 cm, which is around an inch. This will roughly mean that
% angle between the beginning and end of a segment is 90 degrees.
% NOTE: This does not limit the angle by which the segment can bend to 90
% degrees, but makes the error saturate at roughly 90 degrees, what is a
% sane choice. So if a larger curvature was requested and possible, it will
% still be driven to that value.
threshold = inf;
c = 127/40;
     
% Prepare the command packet. K is the vector of curvatures of segment
% which effectively chooses which motor to use to control the position. The
% (measured-target) error is multiplied by (-1).^(K-1) because of the way
% the motors expect the sign of errors to be supplied.
K = arrayfun(@(t,m) select_curvature(t,m), target, measured);
E = c * (target-measured);
command = [K; E];

% Expand the command packed along its rows. This is the format which is
% used by serial port to communicate the error command.
command = int8( reshape(command,1,[]) );

% Send the command
fwrite(controller, command, 'int8');

% Output the input and output data.
% fprintf('mes: ');
% fprintf('%9.2f',measured);
% 
% fprintf('\ntgt: ');
% fprintf('%9.2f',target);
% 
% fprintf('\nk:   ');
% fprintf('%9d',K);
% 
% fprintf('\nerr: ');
% fprintf('%9d', int8(E));
% fprintf('\n\n')

end
