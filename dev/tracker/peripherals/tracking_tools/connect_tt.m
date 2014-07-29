function [ natnet_client, current_frame ] = connect_tt( )
%CONNECT_TT Connect to Tracking Tools
%   Connects to the Tracking Tools program. TT need to have OptiTrack
%   Streaming Engine enabled in unicast mode. It is assumed that the server
%   is on the same machine as the MATLAB client.

% Make .NET assembly visible to MATLAB
NET.addAssembly('C:\Users\manipulator\Arm\tracker\peripherals\tracking_tools\NatNetSDK\lib\x64\NatNetML.dll');

% Create a NatNetML client object, the argument 0 is for multicast mode, 1
% would connect to NatNetML server in unicast mode
natnet_client = NatNetML.NatNetClientML(0);

% Connect to a NatNet server, assume that the server is on the same machine
% so use IP of the local machine
ip = char(java.net.InetAddress.getLocalHost.getHostAddress);
return_code = natnet_client.Initialize(ip, ip);
if (return_code == 0)
    disp('Initialization Succeeded.');
else
    disp('Error Initializing.');
end

% Create the current frame object
current_frame = natnet_client.GetLastFrameOfData();

end
