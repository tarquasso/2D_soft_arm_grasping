function [ segs, spos, old_markers, error ] = get_segments( ...
    raw_markers, spos, srot, lengths, old_markers )

%GET_SEGMENTS Calculates the segments based on marker positions
%   The function operates in two modes:
%
%   [ segs ] = calculate_segments( raw_markers, start_pt, start_rot, lengths, old_markers )
%
%   [ segs ] = calculate_segments( raw_markers, start_pt, start_rot, lengths, NaN )
%
%   raw_markers 2xM matrix - positions of M detected markers
%   spos        2x1 matrix - position of the root of the arm
%   srot            scalar - rotation of the root of the arm
%   lengths     1xS matrix - lengths of arm segments
%   old_markers 2xS matrix - positions of old markers

% Initialize output arguments
segs = NaN;

% Get the number of segments to calculate
S = size(lengths, 2);

% Check #1 - is there a sufficient number of detected markers?
if size(raw_markers, 2) < S+1
    error = 'not_enough_markers';
    return
end

% Prepare the matrix for storing segment properties
segs = NaN(6,S);

% Find start marker
[spos, raw_markers] = find_closest(spos, raw_markers);


% If we are given the old marker positions, assign the markers in the
% current frame based on the previous marker positions. Keep track of
% infinitesimal changes in the position in the vector DS to make sure that
% they aren't too large.
if ~isnan(old_markers)
    DS = NaN(1,S);
    for s=1:S
        [segs(2:3,s), raw_markers, DS(s)] = find_closest(old_markers(:,s), raw_markers);
    end
end

% Assign starting values for the first virtual segment
pos1 = spos;
rot1 = srot;

% Calculate the curvatures and angles using the 2 points method
for s = 1:S
    if isnan(old_markers)
        % Find the next point's expected position
        e = pos1 + [cos(rot1); sin(rot1)] .* lengths(s);
    
        % Find the point closest to the expected position
        [pos2, raw_markers, ~] = find_closest(e, raw_markers);
    else
        % Get the end point of the current segment
        pos2 = segs(2:3,s);
    end
    
    % Calculate properties of the current segment
    [k, rot2] = get_arc(pos1, rot1, pos2);
    arclen = wrapToPi(rot2 - rot1) / k;

    % Save properties of the current segment
    segs(:,s) = [rot1; pos2; rot2; k; arclen];
    
    % Check #2 - is the calculated length more than 10% different than the
    % expected length?
    if abs(arclen - lengths(s)) / lengths(s) > 0.20
        error = 'bad_arclength';
        return
    end
    
    % Prepare properties for the next segment
    pos1 = pos2;
    rot1 = rot2;
end

% Return old markers
old_markers = segs(2:3,:);

% No error
error = NaN;

end
