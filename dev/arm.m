classdef arm
    %ARM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        theta0 = pi/2;           % is the current/measured initial orientation of the first segment
        L = [];                  % is the current/measured length vector <--- MEASURE FROM MOCAP INITIAL FRAME
        N = 6;                   % is the total number of arm segments
        k_min;                   % minimum allowable curvature
        k_max;                   % maximum allowable curvature
        
    end
    
    methods
    end
    
end

