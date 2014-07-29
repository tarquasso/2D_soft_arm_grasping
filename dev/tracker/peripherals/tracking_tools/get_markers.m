function [ markers, timestamp ] = get_markers( frame, fly_zone, vrange )
%GET_MARKERS Returns markers visible in the frame within specified range
%   This function gets markers from a frame returned by Tracking Tools. The
%   coordinate translation between the frame XYZ returned by Tracking Tools
%   and xyz used by the program is the following:
%
%   x = X
%   y = -Z
%   z = Y

% Get timestamp #1
timestamp = frame.fLatency;

% Get the number of objects in the frame
N = frame.nOtherMarkers;

% Prepare a matrix for markers
markers = NaN(2,N);

% Add markers one by one
for i = 1:N
    marker = frame.OtherMarkers.GetValue(i-1);

    y = marker.y;
    
    % Get only markers within the vertical range
    if y > vrange(1) && y < vrange(2)
        markers(:,i) = [ marker.x -marker.z ];
    end
    
    % Check if the timestamp has changed
    if frame.fLatency ~= timestamp
        markers = NaN;
        return
    end
end

% Get only markers within the plane range
IN = inpolygon(markers(1,:), markers(2,:), fly_zone(1,:), fly_zone(2,:));
markers = markers(:,IN);

end

