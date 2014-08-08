classdef Sensor < handle
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        frameRate
        natNetClient
        frameListener
        listenerAttached
        frameOfData
        
        positionData;
        positionTime;
        eulAngles;
    end
    properties(Access=private)
        arm2D
        roundObject
        armController2D          % handle to parent class
    end
    
    methods(Access = public)
        % Constructor
        function obj = Sensor(arm2DHandle,roundObjectHandle,armController2DHandle)
            %Assign Handles
            obj.arm2D = arm2DHandle;
            obj.roundObject = roundObjectHandle;
            obj.armController2D = armController2DHandle;
            
            % Add NatNet .NET assembly so that Matlab can access its methods
            obj.createNatNetClient();
            % Connect to an OptiTrack server (Motive)
            obj.connectToOptiTrack();
            % print out a list of the active tracking Models in Motive
            obj.getTrackedDataDescriptions()
            % get frame rate of tracking system
            obj.getFrameRate();
        end
        function start(obj)
            % setup callback triggered whenever a new frame is received
            obj.setupFrameCallback()
        end
        % Destructor
        function delete(obj)
            % cleanup
            %detach Frame Callback
            obj.detachFrameCallback();
            
            %Uninitialize natNetClient    
            if(~isempty(obj.natNetClient))
                obj.natNetClient.Uninitialize();
                display('[Sensor] Uninitialize NatNetClient.')
            end
        end
    end
    methods(Access = private)
        function createNatNetClient(obj)
            
            % Add NatNet .NET assembly so that Matlab can access its methods, delegates, etc.
            % Note : The NatNetML.DLL assembly depends on NatNet.dll, so make sure they
            % are both in the same folder and/or path if you move them.
            display('[Sensor] Creating NatNetClient.')
            % TODO : update the path to your NatNetML.DLL file here
            %dllPath = fullfile('c:','NatNetSDK2.5','lib','x64','NatNetML.dll');
            curDir = pwd;
            dllPath = fullfile(curDir,'..','3rd','NatNet_SDK_2.6','lib','x64','NatNetML.dll');
            % Check that the file exists
            assert(exist(dllPath, 'file') == 2, 'File does not exist');
            
            %dllPath = fullfile('C:\Users\sandwich\Desktop\3rd\NatNetSDK2.5\lib\x64\NatNetML.dll');
            assemblyInfo = NET.addAssembly(dllPath);
            
            % Create an instance of a NatNet client
            obj.natNetClient = NatNetML.NatNetClientML(0); % Input = iConnectionType: 0 = Multicast, 1 = Unicast
            
            version = obj.natNetClient.NatNetVersion();
            fprintf( '[Sensor] Client Version : %d.%d.%d.%d\n', version(1), version(2), version(3), version(4) );
        end
        
        function connectToOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            display('[Sensor] Connecting to OptiTrack Server (Motive).')
            hst = java.net.InetAddress.getLocalHost;
            HostIP = char(hst.getHostAddress);
            %HostIP = char('128.30.27.47');
            flg = obj.natNetClient.Initialize(HostIP, HostIP); % Flg = returnCode: 0 = Success
            if (flg == 0)
                display('[Sensor] Connection to Server established')
            else
                display('[NatNet] Connection to Server failed')
            end
        end
        
        % Print out a description of actively tracked models from Motive
        function getTrackedDataDescriptions(obj)
            
            %read data descriptions from nat net client
            dataDescriptions = obj.natNetClient.GetDataDescriptions();
            
            % print out
            fprintf('[Sensor] Tracking Models : %d\n\n', dataDescriptions.Count);
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
        
        function getFrameRate(obj)
            % send command/request to Motive to receive frame rate
            [byteArray, retCode] = obj.natNetClient.SendMessageAndWait(...
                'FrameRate');
            if(retCode == 0)
                byteArray = uint8(byteArray);
                obj.frameRate = typecast(byteArray,'single');
                fprintf('[Sensor] FrameRate: %i\n',obj.frameRate);
            else
                fprintf('[Sensor] FrameRate not detected\n');
            end
            
        end
        
        function setupFrameCallback(obj)
            % get the mocap data
            % approach 3 : get data by event handler (no polling)
            % Add NatNet FrameReady event handler
            if(isempty(obj.frameListener))
                obj.frameListener = addlistener(obj.natNetClient,...
                    'OnFrameReady2',@(src,event)frameReadyCallback(obj,src,event));
                display('[Sensor] FrameReady Listener added.');
            else
                display('[Sensor] FrameReady Listener was already added before.');
            end
        end
        function detachFrameCallback(obj)
            if(~isempty(obj.frameListener))
                delete(obj.frameListener);
                obj.frameListener = [];
                display('[Sensor] FrameReady Listener deleted.');
            end          
        end
        
        % Test : Process data in a NatNet FrameReady Event listener callback
        function frameReadyCallback(obj,src,event)
            persistent countDots;
            
            if isempty(countDots)
                countDots = 1;
            end
            
            obj.frameOfData = event.data;
            
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
            obj.extractPositionData();
        end
        
        function extractPositionData(obj)
            
            persistent lastFrameTime;
            persistent lastFrameID;
            persistent arrayIndex;
            persistent bufferModulo;
            
            % first time - generate an array and a plot
            if isempty(lastFrameTime)
                % initialize statics
                bufferModulo = 256;
                arrayIndex = 1;
                lastFrameTime = obj.frameOfData.fLatency;
                lastFrameID = obj.frameOfData.iFrame;
            end
            
            % calculate the frame increment based on mocap frame's timestamp
            % in general this should be monotonically increasing according
            % To the mocap framerate, however frames are not guaranteed delivery
            % so to be accurate we test and report frame drop or duplication
            newFrame = true;
            droppedFrames = false;
            frameTime = obj.frameOfData.fLatency;
            
            frameID = obj.frameOfData.iFrame;
            if(frameID ==lastFrameID)
                %debug
                %fprintf('same id\n');
            end
            
            calcFrameInc = round( (frameTime - lastFrameTime) * obj.frameRate );
            %fprintf('%.12f frameTime\n',frameTime);
            %fprintf('%.12f lastFrameTime\n',lastFrameTime);
            
            % clamp it to a circular buffer of 255 frames
            arrayIndex = mod(arrayIndex + calcFrameInc, bufferModulo);
            if(arrayIndex==0)
                arrayIndex = 1;
            end
            if(calcFrameInc > 1)
                % debug
                %fprintf('\nDropped Frame(s) : %d\n\tLastTime : %.3f\n\tThisTime : %.3f\n', calcFrameInc-1, lastFrameTime, frameTime);
                droppedFrames = true;
            elseif(calcFrameInc == 0)
                % debug
                % fprintf('Duplicate Frame\n')
                newFrame = false;
            end
            
            % debug
            % fprintf('FrameTime: %0.3f\tFrameID: %d\n',frameTime, frameID);
            
            try
                if(newFrame)
                    
                    if(obj.frameOfData.nRigidBodies == 7)
                        % RigidBodyData with properties:
                        %
                        %            ID: 1
                        %             x: 0.1762
                        %             y: 0.0186
                        %             z: -0.0620
                        %            qx: -1.6575e-04
                        %            qy: -1.2448e-04
                        %            qz: 3.9440e-05
                        %            qw: 1
                        %      nMarkers: 4
                        %       Markers: [1x1 NatNetML.Marker[]]
                        %     MeanError: 7.6300e-06
                        %       Tracked: 0
                        
                        %steps to add:
                        % read out only as many rigid bodies as there are: 1000
                        % add method to convert quaternion of each rigid body to an angle
                        % find relevant angle
                        % understand how the center of the rigid body is calculated
                        % find foward and inverse kinematics algorithm to use in
                        % calculating kappa and L
                        % populate the matrix with information
                        
                        %obj.eulAngles = Sensor.extractAnglesFromBody( obj.frameOfData.RigidBodies(1) );
                        
                        x_off = obj.frameOfData.RigidBodies(1).x;
                        y_off = obj.frameOfData.RigidBodies(1).y;
                        z_off = obj.frameOfData.RigidBodies(1).z;
                        
                        %                 positionData = [frameOfData.RigidBodies(2).x-x_off, -(frameOfData.RigidBodies(2).z-z_off), frameOfData.RigidBodies(2).y-y_off, ...
                        %                                 frameOfData.RigidBodies(3).x-x_off, -(frameOfData.RigidBodies(3).z-z_off), frameOfData.RigidBodies(3).y-y_off, ...
                        %                                 frameOfData.RigidBodies(4).x-x_off, -(frameOfData.RigidBodies(4).z-z_off), frameOfData.RigidBodies(4).y-y_off, ...
                        %                                 frameOfData.RigidBodies(5).x-x_off, -(frameOfData.RigidBodies(5).z-z_off), frameOfData.RigidBodies(5).y-y_off];
                        obj.positionData = [x_off,y_off,z_off];
                        obj.positionTime = frameTime * obj.frameRate;
                        
                        % Update Arm
                        
                        % Update Grippper
                        % k and L of Gripper
                        % Indicate to Arm Controller that measurements are
                        % done
                        %obj.armController2D.sensorMeasurementsDone();
                    else
                        fprintf('Only %i Rigid Bodies, but we need 7!\n',obj.frameOfData.nRigidBodies);
                        
                    end
                end
            catch err
                display(err);
            end
            
            lastFrameTime = frameTime;
            lastFrameID = frameID;
            
        end
    end
    methods(Static)
        function angles = extractAnglesFromBody( rigidBody )
            q = quaternion( rigidBody.qx, rigidBody.qy, ...
                rigidBody.qz, rigidBody.qw );
            qRot = quaternion( 0, 0, 0, 1);     % rotate pitch 180 to avoid 180/-180 flip for nicer graphing
            q = mtimes(q, qRot);
            angles = EulerAngles(q,'zyx');
            angleX = -angles(1) * 180.0 / pi;   % must invert due to 180 flip above
            angleY = angles(2) * 180.0 / pi;
            angleZ = -angles(3) * 180.0 / pi;   % must invert due to 180 flip above
            angles = [angleX,angleY,angleZ];
        end
        
        
    end
    
end



