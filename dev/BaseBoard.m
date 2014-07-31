classdef BaseBoard < handle
    %BASEBOARD Describes the board on top of which the experiment occurs
    %   Detailed explanation goes here
    
    properties
        boardDims
    end
    
    methods
        function obj = BaseBoard( )
            %GET_DIMENSIONS Returns all dimensions of the setup in a struct
            %   Detailed explanation goes here
            
            % The structure holding all the data
            obj.boardDims = struct();
            
            % Corners of the board
            obj.boardDims.corners = [
                570.2590  570.2590;
                -26.9410  570.2590;
                -26.9410  -26.9410;
                570.2590  -26.9410 ]' .* unitsratio('m', 'mm');
            
            % Display range of the graph
            obj.boardDims.axis = [min(obj.boardDims.corners(1,:)) max(obj.boardDims.corners(1,:)) ...
                min(obj.boardDims.corners(2,:)) max(obj.boardDims.corners(2,:))];
            
            % Experimental setup range - the "fly zone"
            corner_gap = 3 * 26.9410 * unitsratio('m', 'mm');
            side_gap = 1 * unitsratio('m', 'cm');
            
            fly_zone = [
                obj.boardDims.corners(:,1) + [-side_gap; -corner_gap], ...
                obj.boardDims.corners(:,1) + [-corner_gap; -side_gap], ...
                ...
                obj.boardDims.corners(:,2) + [corner_gap; -side_gap], ...
                obj.boardDims.corners(:,2) + [side_gap; -corner_gap], ...
                ...
                obj.boardDims.corners(:,3) + [side_gap; corner_gap], ...
                obj.boardDims.corners(:,3) + [corner_gap; side_gap], ...
                ...
                obj.boardDims.corners(:,4) + [-corner_gap; side_gap], ...
                obj.boardDims.corners(:,4) + [-side_gap; corner_gap], ...
                ] ;
            
            obj.boardDims.fly_zone = [fly_zone, fly_zone(:,1)];
            
            obj.boardDims.vrange = [0 0.1];
            
        end
    end
    
end

