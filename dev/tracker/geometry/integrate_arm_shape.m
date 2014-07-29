function [ curve ] = integrate_arm_shape( spos, segs )
%INTEGRATE_ARM_SHAPE Integrates the position of the arm
%   This function uses the segments matrix to integrate the shape of the
%   arm so that it cam be displayed on a graph.
%   
%   The function requires:
%
%   spos    2x1 vector  starting position of the arm
%   segs    6xS matrix  segments description
%
%   The only needed elements of the segs matrix are:
%
%       segs(1,1)   - starting rotation
%       segs(5,:)   - curvatures
%       segs(6,:)   - arclengths

% Get information about the arm
S = size(segs,2);   % Number of segments

% Set up integration properties
n = 100;             % Number of steps per segment
N = (S-1)*n+1;      % Total number of steps           

% Preallocate space for the curve
curve = NaN(2,N);
curve(:,1) = spos;

% First starting point
pos1 = spos;
rot1 = segs(1,1);
i = 2;

for s = 1:S
    % Set simulation parameters
    ds = segs(6,s) / n;
    drot = ds * segs(5,s);
    
    % Iterate l times in one segment
    for j = 1:n
        pos2 = pos1 + [cos(rot1); sin(rot1)] * ds;
        curve(:, i) = pos2;
        i = i + 1;
        
        pos1 = pos2;
        rot1 = rot1 + drot;
    end
end

% for i = 1:N
%     % Set matrix offset
%     offset = (i-1) * n + 1;
%     
%     % Set simulation parameters
%     ds = segs(i,3) / n;
%     dorient = wrapToPi(segs(i,6) - segs(i,5)) / n;
%     
%     for j = 1:n
%         % Calculate current point
%         this_point = prev_point + [cos(prev_rot); sin(prev_rot)] * ds;
%         curve(offset+j,:) = this_point';
%         
%         % Update point for the next iteration
%         prev_point = this_point;
%         prev_rot = prev_rot + dorient;
%     end
%     
%     % Next starting point if it is not the last iteration
%     if i < S
%         prev_point = segs(i,1:2)';
%         prev_rot = segs(i+1,5);
%     end
% end

end
