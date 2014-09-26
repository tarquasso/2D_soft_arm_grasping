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
        kMeas
        arcLenMeas
        kTarget
        tipPosition
        objectPosition
    end
    
    methods
        function obj = ShapeHistory(S, N)
            % Create the initial object
            obj.S = S;
            obj.N = N;
            obj.i = 1;
            
            obj.timestamps = NaN(N,1);
            obj.kMeas = NaN(N,S);
            obj.arcLenMeas = NaN(N,S);
            obj.kTarget = NaN(N,S);
            obj.tipPosition = NaN(N,3);
            obj.objectPosition = NaN(N,2);
            
        end
        
        function add(obj, timestamp, kMeas, arcLenMeas, kTarget, tipPosition, objectPosition)
            % Add a new measurement
            obj.timestamps(obj.i, 1) = timestamp;
            obj.kMeas(obj.i, :) = kMeas;
            obj.arcLenMeas(obj.i, :) = arcLenMeas;
            obj.kTarget(obj.i, :) = kTarget;
            obj.tipPosition(obj.i, :) = tipPosition;
            obj.objectPosition(obj.i, :) = objectPosition;

            % Update the count
            obj.i = mod(obj.i + 1, obj.N);
            if obj.i == 0
                obj.i = obj.N;
            end
        end
        
        %Destructor
        function delete(obj)
        end
        
    end
    
end
