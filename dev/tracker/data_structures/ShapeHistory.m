classdef ShapeHistory < handle
    %SHAPEHISTORY Class containing the history of the arm's shape
    %   This is a handle class, i.e. objects are always passed by reference
    %   rather than value. As a result the values can be efficiently
    %   updated on the go without copying the whole history.
    
    properties
        S   % Number of segments in the arm
        N   % Size of samples buffer
        i   % Current index 
        
        timestamps      % Timestamps of the state 
        start_pts       % Positions of the start point
        shapes          % Shapes of the segments
        target_curvatures          % filtered curvatures
        cmds % commands sent to cc
    end
    
    methods
        function obj = ShapeHistory(S, N)
            % Create the initial object
            obj.S = S;
            obj.N = N;
            obj.i = 1;
            
            obj.timestamps = NaN(1,N);
            obj.shapes = NaN(6,S,N);
            obj.start_pts = NaN(2,N);
            obj.target_curvatures = NaN(6,N);
        end
        
        function add(obj, timestamp, spos, segments, target, cmd)
            % Add a new measurement
            obj.timestamps(obj.i) = timestamp;
            obj.start_pts(:,obj.i) = spos;
            obj.shapes(:,:,obj.i) = segments;
            obj.target_curvatures(:, obj.i) = target;
            obj.cmds(obj.i, :) = cmd;
            
            % Update the count
            obj.i = mod(obj.i + 1, obj.N);
            if obj.i == 0
                obj.i = obj.N;
            end
        end
        
        function timestamp = timestamp(obj, i)
            % Returns start points indexing from the end 
            timestamp = obj.timestamps(obj.i - i);
        end
        
        function st_point = st_point(obj, i)
            % Returns start points indexing from the end 
            st_point = obj.st_points(obj.i - i);
        end
        
        function shape = shape(obj, i)
            % Returns start points indexing from the end 
            shape = obj.shapes(obj.i - i);
        end
        
        function target_curvature = target_curvature(obj, i)
            % Returns start points indexing from the end 
            target_curvature  = obj.target_curvatures(obj.i - i);
        end
        
        function cmd = cmd(obj, i)
            % Returns start points indexing from the end 
            cmd  = obj.cmds(obj.i - i);
        end
    end
    
end
