classdef TrajGen < handle
    %TRAJGEN Summary
    
    properties
    end
    
    methods
        function obj = TrajGen()
        end
        
        %Generate a feasible Cartestian velocity profile given a desired distance to move
        function [velocityProfile, tf] = generateVelocityProfile(obj, k_i, k_f, vMax, aMax )
            
            d = k_f-k_i;
            
            if( abs(d) < 2*(vMax^2/aMax) )
                velocity = sign(d)*sqrt(abs(d)*aMax);
                t1 = abs(d)/abs(velocity);
                t2 = t1;
            else
                velocity = sign(d)*vMax;
                t1 = abs(velocity)/aMax;
                t2 = abs(d)/abs(velocity);
            end
            
            tf = t1+t2;
            
            velocityProfile = PlannerGrasp.firstOrderHold([0, t1, t2, tf, tf+0.1], [0, velocity, velocity, 0, 0]);
        end
        
        %Get a feasible Cartesian position delta based on a velcoity
        function PositionDelta = getPositionDelta(obj, velocityProfile, tCurrent, k_i)
            %tCurrent is the time along a linear path starting at t=0
            PositionDelta = integral( @(x)ppval(velocityProfile,x), 0, (tCurrent + 0.025) ) + k_i;
        end  
        
        function delete(obj)
            
        
        end   
    end
    
    
end