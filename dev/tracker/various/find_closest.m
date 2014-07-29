function [ closest, points, error ] = find_closest( ref, points )
%FIND_CLOSEST Returns the closest point from an array of points
%   ref is a 2x1 vector and points is an Nx2 matrix

[~, index] = min(sum((points - repmat(ref, 1, size(points,2))).^2,1));
closest = points(:, index);
left = true(1,size(points,2));
left(index) = false;
points = points(:,left);
error = norm(closest-ref);

end
