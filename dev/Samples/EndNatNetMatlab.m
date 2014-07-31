
% Optitrack Matlab / NatNet Sample
% 
%  Requirements:
%   - OptiTrack Motive 1.5 or later
%   - OptiTrack NatNet 2.5 or later
%   - Matlab R2013
%

function EndNatNetMatlab(theClient,ls)

    % cleanup
    theClient.Uninitialize();
    if(~isempty(ls))
            delete(ls);
    end
    clear functions;

    display('NatNet Sample End')
    
end


 