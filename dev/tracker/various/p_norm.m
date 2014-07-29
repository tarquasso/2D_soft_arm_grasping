function [ n ] = p_norm( v1, v2, p )
%P_NORM Returns the p-norm of the difference between two vectors
%   Detailed explanation goes here

n = sum(abs(v1-v2).^p).^(1/p);

end

