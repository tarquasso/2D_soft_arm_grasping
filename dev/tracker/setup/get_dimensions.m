function [ dims ] = get_dimensions( model )
%GET_DIMENSIONS Returns all dimensions of the setup in a struct
%   Detailed explanation goes here

% The structure holding all the data
dims = struct();

% Corners of the board
dims.corners = [ 
    570.2590  570.2590;
    -26.9410  570.2590;
    -26.9410  -26.9410;
    570.2590  -26.9410 ]' .* unitsratio('m', 'mm');

% Display range of the graph
dims.axis = [min(dims.corners(1,:)) max(dims.corners(1,:)) ...
    min(dims.corners(2,:)) max(dims.corners(2,:))];

% Experimental setup range - the "fly zone"
corner_gap = 3 * 26.9410 * unitsratio('m', 'mm');
side_gap = 1 * unitsratio('m', 'cm');

fly_zone = [
    dims.corners(:,1) + [-side_gap; -corner_gap], ...
    dims.corners(:,1) + [-corner_gap; -side_gap], ...
    ...
    dims.corners(:,2) + [corner_gap; -side_gap], ...
    dims.corners(:,2) + [side_gap; -corner_gap], ...
    ...
    dims.corners(:,3) + [side_gap; corner_gap], ...
    dims.corners(:,3) + [corner_gap; side_gap], ...
    ...
    dims.corners(:,4) + [-corner_gap; side_gap], ...
    dims.corners(:,4) + [-side_gap; corner_gap], ...
    ] ;

dims.fly_zone = [fly_zone, fly_zone(:,1)];

dims.vrange = [0 0.1];

% Dimensions and orientation of the arm
if strcmpi(model,'A')
    dims.S = 6;
    dims.lengths = repmat(1.5,1,dims.S) .* unitsratio('m','inch');
    dims.spos = [264.3; 188.3] .* unitsratio('m','mm');
    dims.srot= pi/2;
end

if strcmpi(model,'B')
    dims.S = 6;
    dims.lengths = repmat(2.37,1,dims.S) .* unitsratio('m','inch');
    dims.spos = [263.8; 152.7] .* unitsratio('m','mm');
    dims.srot= pi/2;
end

end
