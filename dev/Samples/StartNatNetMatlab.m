
function [theClient, ls] = StartNatNetMatlab()

    display('NatNet Sample Begin')
    
    global frameRate;
    
      % Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
    % Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
    % are both in the same folder and/or path if you move them.
    display('[NatNet] Creating Client.')
    % TODO : update the path to your NatNetML.DLL file here
    %dllPath = fullfile('c:','NatNetSDK2.5','lib','x64','NatNetML.dll');
    curDir = pwd;
    dllPath = fullfile(curDir,'..','3rd','NatNet_SDK_2.6','lib','x64','NatNetML.dll');
    % Check that the file exists
    assert(exist(dllPath, 'file') == 2, 'File does not exist');

        
    %dllPath = fullfile('C:\Users\sandwich\Desktop\3rd\NatNetSDK2.5\lib\x64\NatNetML.dll');
    assemblyInfo = NET.addAssembly(dllPath);

    % Create an instance of a NatNet client
    theClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
    version = theClient.NatNetVersion();
    fprintf( '[NatNet] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );

    % Connect to an OptiTrack server (e.g. Motive)
    display('[NatNet] Connecting to OptiTrack Server.')
    hst = java.net.InetAddress.getLocalHost;
    HostIP = char(hst.getHostAddress);
    %HostIP = char('128.30.27.47');
    flg = theClient.Initialize(HostIP, HostIP); % Flg = returnCode: 0 = Success
    if (flg == 0)
        display('[NatNet] Initialization Succeeded')
    else
        display('[NatNet] Initialization Failed')
    end

    % print out a list of the active tracking Models in Motive
    GetDataDescriptions(theClient)

    % Test - send command/request to Motive
    [byteArray, retCode] = theClient.SendMessageAndWait('FrameRate');
    if(retCode ==0)
        byteArray = uint8(byteArray);
        frameRate = typecast(byteArray,'single');
    end
    fprintf('frameRate: %i\n',frameRate);
    % get the mocap data
    % approach 3 : get data by event handler (no polling)
    % Add NatNet FrameReady event handler
    ls = addlistener(theClient,'OnFrameReady2',@(src,event)FrameReadyCallback(src,event));
    display('[NatNet] FrameReady Listener added.');
end

% Test : Process data in a NatNet FrameReady Event listener callback
function FrameReadyCallback(src, event)
    persistent countDots;
    global frameOfData; %for debug made global

    if isempty(countDots)
        countDots = 1;
    end
    
    frameOfData = event.data;
    getPositionData( frameOfData );
    
    % Code to display if framedata has been received
    if(false)
    fprintf('.');
    if countDots > 45
    fprintf('\n'); % That \n explicitly adds the linefeed
    countDots = 1;
    else
    countDots = countDots+1;
    end
    end
    % do some work on that pass...
    
end

% Print out a description of actively tracked models from Motive
function GetDataDescriptions( theClient )

    dataDescriptions = theClient.GetDataDescriptions();
    
    % print out 
    fprintf('[NatNet] Tracking Models : %d\n\n', dataDescriptions.Count);
    for idx = 1 : dataDescriptions.Count
        descriptor = dataDescriptions.Item(idx-1);
        if(descriptor.type == 0)
            fprintf('\tMarkerSet \t: ');
        elseif(descriptor.type == 1)
            fprintf('\tRigid Body \t: ');                
        elseif(descriptor.type == 2)
            fprintf('\tSkeleton \t: ');               
        else
            fprintf('\tUnknown data type : ');               
        end
        fprintf('%s\n', char(descriptor.Name));
    end

    for idx = 1 : dataDescriptions.Count
        descriptor = dataDescriptions.Item(idx-1);
        if(descriptor.type == 0)
            fprintf('\n\tMarkerset : %s\t(%d markers)\n', char(descriptor.Name), descriptor.nMarkers);
            markerNames = descriptor.MarkerNames;
            for markerIndex = 1 : descriptor.nMarkers
                name = markerNames(markerIndex);
                fprintf('\t\tMarker : %-20s\t(ID=%d)\n', char(name), markerIndex);             
            end
        elseif(descriptor.type == 1)
            fprintf('\n\tRigid Body : %s\t\t(ID=%d, ParentID=%d)\n', char(descriptor.Name),descriptor.ID,descriptor.parentID);
        elseif(descriptor.type == 2)
            fprintf('\n\tSkeleton : %s\t(%d bones)\n', char(descriptor.Name), descriptor.nRigidBodies);
            %fprintf('\t\tID : %d\n', descriptor.ID);
            rigidBodies = descriptor.RigidBodies;
            for boneIndex = 1 : descriptor.nRigidBodies
                rigidBody = rigidBodies(boneIndex);
                fprintf('\t\tBone : %-20s\t(ID=%d, ParentID=%d)\n', char(rigidBody.Name), rigidBody.ID, rigidBody.parentID);
            end               
        end
    end

end


 