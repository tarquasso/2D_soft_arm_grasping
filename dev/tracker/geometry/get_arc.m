function [ k, erot ] = get_arc( spos, srot, epos )
%GET_ARC Calculates the curvature of a segment between spos and epos
%
%   INPUT:
%   spos    2x1 vector - start position
%   srot        scalar - start rotation
%   epos    2x1 vector - end position
%
%   OUTPUT:
%   k           scalar - signed curvature between spos and epos
%   erot        scalar - end rotation
%
%   The method involves constructing two lines which are known to be
%   perpendicular to the arc:
%
%       u - which is perpendicular to the arc at point spos (due to known
%           srot)
%       v - which is perpendicular to the arc at the point exactly between
%           spos and epos (the perpendicular bisector of the segment
%           between them)
%
%   Their intersection gives as the position of the center of curvature of
%   the arc, from which the radius and curvature k = r^-1 can be
%   calculated. The curvature is signed according to "right hand screw
%   rule".

% Parametric equation of the first line
u = [spos [cos(srot + pi/2); sin(srot + pi/2)]];

% Parametric equation of the second line
v = [(spos + epos) ./ 2 [0 -1; 1 0] * (epos - spos)];

% Value of the second line's parameter at the intersection of the two lines
lambda_v = ( u(2,2) * (v(1,1) - u(1,1)) - u(1,2) * (v(2,1) - u(2,1)) ) /...
           ( u(1,2) * v(2,2) - u(2,2) * v(1,2) );

if lambda_v == Inf
    % Center of curvature at an infinite distance, zero curvature
    k = 0;
    
    % Out vector
    out_vector = epos - spos;
else
    % Intersection coordinates
    o = v(:,1) + lambda_v .* v(:,2);

    % Cross product of (a-o) and (b-o) in the third dimension
    e = spos-o;
    f = epos-o;
    dir = e(1)*f(2) - e(2)*f(1);

    % Signed curvature
    k = sign(dir) * sum((epos - o).^2)^-0.5;

    % Cross product of a-c and the direction 'vector'
    out_vector = dir * [-f(2); f(1)];
end

% End rotation
erot = atan2(out_vector(2), out_vector(1));

%      if(erot <= 0.0)
%         erot = 2*pi + erot; 
%      end

end
