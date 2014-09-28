classdef RoundObject < handle
    
    properties(SetAccess=private,GetAccess=public)
        x;          % x coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
        y;          % y coordinate of the center of the object  <--- MEASURE FROM MOCAP INITIAL FRAME
        r;          % r is the radius of the circle             <--- MEASURE FROM MOCAP INITIAL FRAME
    end
    
    properties
        placed;
    end
    
    methods
        %Constructor
        function obj = RoundObject()
            obj.placed = 0;
            obj.r = 0.0324/2; %Radius initialized by construction [m]
        end
        %Destructor
        function delete(obj)
            
        end
        %Set measured state of the object
        function setMeasuredState(obj, val_x, val_y)
            obj.x = val_x;
            obj.y = val_y;
        end
        %Plot the object and return a handle to the plot
        function h = plotObjectToHandle(obj, objectColor)
            if nargin < 2
                objectColor = 'g'; 
            end
            
            h = rectangle('Position',[(obj.x - obj.r), (obj.y - obj.r), 2*obj.r, 2*obj.r ],'Curvature',[1,1],...
                'FaceColor',objectColor);
            daspect([1,1,1]);
            
%             delta_angle = 0.01;
%             
%             ang = 0:delta_angle:2*pi;
%             xp = obj.r*cos(ang);
%             yp = obj.r*sin(ang);
%             h = plot(obj.x+xp,obj.y+yp, objectColor, 'LineWidth', 1.5);
            
        end
    end
    
end

