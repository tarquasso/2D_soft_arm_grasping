classdef Sensor < handle
    %SENSOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        
        frameRate
        natNetClient
        frameListener
        listenerAttached
        frameOfData
        
        positionTime;
        eulAngles;
    end
    properties(Access=private)
        arm2D
        roundObject
        armController2D          % handle to parent class
        natNetClientInit
        totalNumRigidBodies
    end
    
    methods(Access = public)
        % Constructor
        function obj = Sensor(arm2DHandle,roundObjectHandle,armController2DHandle)
            %Assign Handles
            obj.arm2D = arm2DHandle;
            obj.roundObject = roundObjectHandle;
            obj.armController2D = armController2DHandle;
            % arm plus gripper plus one for the final marker plus one
            % additional marker for the object:
            obj.totalNumRigidBodies = obj.arm2D.dims.S+obj.arm2D.gripper2D.dims.S+1+1; 
            obj.natNetClientInit = false;
            
            % Add NatNet .NET assembly so that Matlab can access its methods
            obj.createNatNetClient();
            % Connect to an OptiTrack server (Motive)
            obj.connectToOptiTrack();
            % print out a list of the active tracking Models in Motive
            %obj.getTrackedDataDescriptions()
            % get frame rate of tracking system
            obj.getFrameRate();
        end
        function start(obj)
            % setup callback triggered whenever a new frame is received
            obj.attachFrameCallback()
        end
        function stop(obj)
            % setup callback triggered whenever a new frame is received
            obj.detachFrameCallback()
        end
        % Destructor
        function delete(obj)
            % cleanup
            %detach Frame Callback
            obj.detachFrameCallback();
            
            %Uninitialize natNetClient
            obj.disconnectFromOptiTrack();
            
            %delete NatNetClient
            obj.destroyNatNetClient();
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
        function destroyNatNetClient(obj)
            if(~isempty(obj.natNetClient))
                obj.natNetClient.delete();
                obj.natNetClient = [];
                display('[Sensor] NatNetClient deleted!')
            else
                error('[Sensor] NatNetClient does not exist!');
            end       
        end
        
        function connectToOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            display('[Sensor] Connecting to OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient)&& obj.natNetClientInit == false)
                hst = java.net.InetAddress.getLocalHost;
                HostIP = char(hst.getHostAddress);
                %HostIP = char('128.30.27.47');
                flg = obj.natNetClient.Initialize(HostIP, HostIP); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = true;
                    display('[Sensor] Connection to Server established')
                else
                    error('[Sensor] Connection to Server failed!')
                end
            else
                error('[Sensor] NatNetClient not created yet!')  
            end
        end
        
        function disconnectFromOptiTrack(obj)
            % Connect to an OptiTrack server (e.g. Motive)
            display('[Sensor] Disconnecting from OptiTrack Server (Motive).')
            if(~isempty(obj.natNetClient) && obj.natNetClientInit == true)
                %Uninitialize natNetClient
                flg = obj.natNetClient.Uninitialize(); % Flg = returnCode: 0 = Success
                if (flg == 0)
                    obj.natNetClientInit = false;
                    display('[Sensor] Succesfully disconnected from NatNetClient.')
                else
                    error('[Sensor] Disconnecting from NatNetClient failed!')
                end
            else
                error('[Sensor] Already disconnected from NatNetClient.');
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
        
        function attachFrameCallback(obj)
            % get the mocap data
            % approach 3 : get data by event handler (no polling)
            % Add NatNet FrameReady event handler
            if(obj.natNetClientInit)
                if(isempty(obj.frameListener))
                    obj.frameListener = addlistener(obj.natNetClient,...
                        'OnFrameReady2',@(src,event)frameReadyCallback(obj,src,event));
                    display('[Sensor] FrameReady Listener added.');
                else
                    display('[Sensor] FrameReady Listener was already added before.');
                end
            else
                error('[Sensor] NatNet Client is not initialized.');
            end
        end
        
        function detachFrameCallback(obj)
            if(obj.natNetClientInit)
                if(~isempty(obj.frameListener))
                    delete(obj.frameListener);
                    obj.frameListener = [];
                    display('[Sensor] FrameReady Listener deleted.');
                else
                    display('[Sensor] FrameReady Listener already deleted.');
                end
            else
                error('[Sensor] NatNet Client is not initialized, can not deattach.');
            end
        end
        
        % Test : Process data in a NatNet FrameReady Event listener callback
        function frameReadyCallback(obj,src,event)
            
            obj.frameOfData = event.data;
            %Sensor.helperDisplayDots(); %for debugging to display dots
            % do some work on that pass...
            obj.extractPositionData();
        end
        
        function extractPositionData(obj)
            
            persistent p_lastFrameTime;
            persistent p_lastFrameID;
            persistent p_arrayIndex;
            persistent p_bufferModulo;
            
            % first time - generate an array and a plot
            if isempty(p_lastFrameTime)
                % initialize statics
                p_bufferModulo = 256;
                p_arrayIndex = 1;
                p_lastFrameTime = double(obj.frameOfData.fLatency);
                p_lastFrameID = obj.frameOfData.iFrame;
            end
            
            % calculate the frame increment based on mocap frame's timestamp
            % in general this should be monotonically increasing according
            % To the mocap framerate, however frames are not guaranteed delivery
            % so to be accurate we test and report frame drop or duplication
            l_newFrame = true;
            l_droppedFrames = false;
            l_frameTime = double(obj.frameOfData.fLatency);
            
            l_frameID = obj.frameOfData.iFrame;
            if(l_frameID ==p_lastFrameID)
                %debug
                %fprintf('same id\n');
            end
            
            l_calcFrameInc = round( (l_frameTime - p_lastFrameTime) * obj.frameRate );
            %fprintf('%.12f l_frameTime\n',l_frameTime);
            %fprintf('%.12f p_lastFrameTime\n',p_lastFrameTime);
            
            % clamp it to a circular buffer of 255 frames
            p_arrayIndex = mod(p_arrayIndex + l_calcFrameInc, p_bufferModulo);
            if(p_arrayIndex==0)
                p_arrayIndex = 1;
            end
            if(l_calcFrameInc > 1)
                % debug
                %fprintf('\nDropped Frame(s) : %d\n\tLastTime : %.3f\n\tThisTime : %.3f\n', l_calcFrameInc-1, p_lastFrameTime, l_frameTime);
                l_droppedFrames = true;
            elseif(l_calcFrameInc == 0)
                % debug
                % fprintf('Duplicate Frame\n')
                l_newFrame = false;
            end
            
            % debug
            % fprintf('l_frameTime: %0.3f\tFrameID: %d\n',l_frameTime, l_frameID);
            
            try
                if(l_newFrame)
                    if(obj.frameOfData.nRigidBodies == obj.totalNumRigidBodies)
                        % RigidBodyData with properties:
                        % ID,x,y,z,qx,qy,qz,qw,nMarkers,Markers,MeanError,Tracked
                        obj.positionTime = double(l_frameTime * obj.frameRate);
                        obj.armController2D.plannerGrasp.framePeriod = double(1/(obj.frameRate));
                        
                        i = 1;
                        j =1;
                        % Extract info for arm and gripper
                        for s = 1:obj.totalNumRigidBodies
                            if (s >= 1 && s <= obj.arm2D.dims.S + 1)
                            %TAKE X position
                            obj.arm2D.segPos2D(1,i) = double(obj.frameOfData.RigidBodies(s).z);
                            %TAKE Y position
                            obj.arm2D.segPos2D(2,i) = double(obj.frameOfData.RigidBodies(s).x);
                            i = i+1;
                            end
                            if ( s >= obj.arm2D.dims.S +1 &&  s <= obj.arm2D.dims.S +1+obj.arm2D.gripper2D.dims.S)
                             %TAKE X position
                            obj.arm2D.gripper2D.segPos2D(1,j) = double(obj.frameOfData.RigidBodies(s).z);
                            %TAKE Y position
                            obj.arm2D.gripper2D.segPos2D(2,j) = double(obj.frameOfData.RigidBodies(s).x);

                            j = j+1;
                            end
                            if (s == obj.arm2D.dims.S+1+obj.arm2D.gripper2D.dims.S+1)
                                l_roundObjectX = double(obj.frameOfData.RigidBodies(s).z-obj.frameOfData.RigidBodies(1).z);
                                l_roundObjectY = double(obj.frameOfData.RigidBodies(s).x-obj.frameOfData.RigidBodies(1).x);
                                obj.roundObject.setMeasuredState(l_roundObjectX,l_roundObjectY);
                            end
                        end
                       
                        
                        % Update Arm and Gripper Values
                        obj.arm2D.calculateSegmentValues();
                   
                        % Tell Arm Controller that measurements are done
                        obj.armController2D.sensorMeasurementsDone();
                    else
                        error('[Sensor] We have %i Rigid Bodies, but we need %i!\n',obj.frameOfData.nRigidBodies,obj.totalNumRigidBodies);
                    end
                end
            catch exc
                getReport(exc, 'extended')
            end
            
            p_lastFrameTime = l_frameTime;
            p_lastFrameID = l_frameID;
            
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
        function helperDisplayDots( )
            persistent countDots;
            if isempty(countDots)
                countDots = 1;
            end
            % Code to display if framedata has been received
            fprintf('.');
            if countDots > 45
                fprintf('\n'); % That \n explicitly adds the linefeed
                countDots = 1;
            else
                countDots = countDots+1;
            end
        end
    end
    
end



