function [x, y, theta] = recursive_forward_kinematics(k, theta0, L, N, i, s)
    % k is the current/measured curvature vector
    % theta0 is the current/measured initial orientation of the first segment
    % L is the current/measured length vector
    % N is the total number of segments
    % i is the segment of interest
    % s is the length of interest along segment i
    
    theta_init = zeros(1,N);
    x_init = zeros(1,N);
    y_init = zeros(1,N);
    
    if( i == 1)
        theta_init(i) = theta0;
        x_init(i) = 0.0;
        y_init(i) = 0.0;
    else
        [x_init(i), y_init(i), theta_init(i)]=recursive_forward_kinematics(k, theta0, L, N, i-1, L(i-1));
    end
    
    theta = theta_init(i) + k(i)*s;
    x = x_init(i) + sin(theta)/k(i) - sin(theta_init(i))/k(i);
    y = y_init(i) - cos(theta)/k(i) + cos(theta_init(i))/k(i);

end