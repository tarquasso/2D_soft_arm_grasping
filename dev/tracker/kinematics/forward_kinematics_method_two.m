function [ x_tip, y_tip, theta_tip ] = forward_kinematics_method_two( k, root_rot, s )
%FORWARD_KINEMATICS Calculates the tip position relative to starting point
%   Detailed explanation goes here

% Replace zero curbatures with slightly perturbed ones (100 times machine
% precision)
k(k==0) = 100*eps;

% Calculate the radii of curvature
r = 1./k';

% Calculate the rotations of every segment. Start by calculating the
% difference between the start and end rotation of every segment (curvature
% * length) and then add them together to get the tip rotations.
d_rot = k'.*s;
t_rot = root_rot + cumsum(d_rot);
s_rot = [root_rot t_rot(1:end-1)];

% Calculate the x segment tip positions in a similar manner
d_x = r.*sin(t_rot) - r.*sin(s_rot);
x = cumsum(d_x);

d_y = -r.*cos(t_rot) + r.*cos(s_rot);
y = cumsum(d_y);

% Return the tip position
x_tip = x(end);
y_tip = y(end);
theta_tip = t_rot(end);

end

