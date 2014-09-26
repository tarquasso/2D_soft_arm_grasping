classdef TrajGen < handle
    %TRAJGEN Summary
    
    properties
        vMax % array of maximum arc space velocities
        aMax % array of maximum arc space accelerations
    end
    
    methods
        function obj = TrajGen()
            %TODO: trajGen should have handle to arm2D to know the
            %dimensions of the arm
            obj.vMax = 3*ones(1,6);
            obj.aMax = 1*ones(1,6);
        end
        
        %Generate multiple feasible Cartestian velocity profiles given a desired distance to move
        function [l_curvatureProfiles, l_maxTrajectoryEndTime] = generateMultipleVelocityProfiles(obj, l_kInitial, l_kTarget )
            
            [l_seg1CurvatureProfile, tf1] = obj.generateVelocityProfile(l_kInitial(1), l_kTarget(1), obj.vMax(1), obj.aMax(1) );
            [l_seg2CurvatureProfile, tf2] = obj.generateVelocityProfile(l_kInitial(2), l_kTarget(2), obj.vMax(2), obj.aMax(2) );
            [l_seg3CurvatureProfile, tf3] = obj.generateVelocityProfile(l_kInitial(3), l_kTarget(3), obj.vMax(3), obj.aMax(3) );
            [l_seg4CurvatureProfile, tf4] = obj.generateVelocityProfile(l_kInitial(4), l_kTarget(4), obj.vMax(4), obj.aMax(4) );
            [l_seg5CurvatureProfile, tf5] = obj.generateVelocityProfile(l_kInitial(5), l_kTarget(5), obj.vMax(5), obj.aMax(5) );
            [l_seg6CurvatureProfile, tf6] = obj.generateVelocityProfile(l_kInitial(6), l_kTarget(6), obj.vMax(6), obj.aMax(6) );
            
            l_curvatureProfiles = [l_seg1CurvatureProfile, l_seg2CurvatureProfile, l_seg3CurvatureProfile...
                            l_seg4CurvatureProfile, l_seg5CurvatureProfile, l_seg6CurvatureProfile];
            l_maxTrajectoryEndTime = max([tf1, tf2, tf3, tf4, tf5, tf6]);
            
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
        %Get multiple feasible Cartesian position deltas based on a velcoity
        function [l_kIntermediate] = generateMultplePositionDeltas(l_curvatureProfiles, l_time, l_kInit)
            
            l_kIntermediate = zeros(1,6);
            l_kIntermediate(1) = obj.getPositionDelta(l_curvatureProfiles(1), l_time, l_kInit(1));
            l_kIntermediate(2) = obj.getPositionDelta(l_curvatureProfiles(2), l_time, l_kInit(2));
            l_kIntermediate(3) = obj.getPositionDelta(l_curvatureProfiles(3), l_time, l_kInit(3));
            l_kIntermediate(4) = obj.getPositionDelta(l_curvatureProfiles(4), l_time, l_kInit(4));
            l_kIntermediate(5) = obj.getPositionDelta(l_curvatureProfiles(5), l_time, l_kInit(5));
            l_kIntermediate(6) = obj.getPositionDelta(l_curvatureProfiles(6), l_time, l_kInit(6));
            
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