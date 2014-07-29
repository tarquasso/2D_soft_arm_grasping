function [X, Y] = getEnvelope(k, L, N, theta0, half_thickness)

    x_col = 1;
    x_row = 1;
    y_col = 2;
    y_row = 2;

    %% define robot's envelop with a set of points
    t1L = zeros(N,2); % points representing the top and bottom middle of each segment
    t1R = zeros(N,2);
    t2L = zeros(N,2); % points representing the top and bottom middle of each segment
    t2R = zeros(N,2);
    eL = zeros(N,2); % points representing the top and bottom end of each segment
    eR = zeros(N,2);
    X = [];
    Y = [];

    b = [0, 0, -1];
    g = 2.0*half_thickness/25;

    for i = 1:N
        [x, y, theta] = recursive_forward_kinematics(k, theta0, L, N, i, L(i)/3);
        a = [cos(theta), sin(theta), 0];
        r = cross(a, b);
        r = [r(1), r(2)];

        if k(i) > 0 
            t1L(i,:) = [x, y] + half_thickness*r;
            t1R(i,:) = [x, y] + -(half_thickness+g*abs(k(i)))*r;
        else
            t1L(i,:) = [x, y] + (half_thickness+g*abs(k(i)))*r;
            t1R(i,:) = [x, y] + -half_thickness*r;
        end
        
        [x, y, theta] = recursive_forward_kinematics(k, theta0, L, N, i, 2*L(i)/3);
        a = [cos(theta), sin(theta), 0];
        r = cross(a, b);
        r = [r(1), r(2)];

        if k(i) > 0 
            t2L(i,:) = [x, y] + half_thickness*r;
            t2R(i,:) = [x, y] + -(half_thickness+g*abs(k(i)))*r;
        else
            t2L(i,:) = [x, y] + (half_thickness+g*abs(k(i)))*r;
            t2R(i,:) = [x, y] + -half_thickness*r;
        end

        [x, y, theta] = recursive_forward_kinematics(k, theta0, L, N, i, L(i));
        a = [cos(theta), sin(theta), 0];
        r = cross(a, b);
        r = [r(1), r(2)];

        eL(i,:) = [x, y] + half_thickness*r;
        eR(i,:) = [x, y] + -half_thickness*r;
    end

    %% Create polgon representation
    for i = 1:N
        X = [X; t1L(i, x_col); t2L(i, x_col); eL(i, x_col)];
        Y = [Y; t1L(i, y_col); t2L(i, y_col); eL(i, y_col)];
    end

    for i = N:-1:1
        X = [X; eR(i, x_col); t2R(i, x_col); t1R(i, x_col)];
        Y = [Y; eR(i, y_col); t2R(i, y_col); t1R(i, y_col)];
    end

    X = [X; X(1)];
    Y = [Y; Y(1)];

end

