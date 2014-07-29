function getPositionData( frameOfData )

    persistent lastFrameTime;
    persistent lastFrameID;
    persistent arrayIndex;
    persistent bufferModulo;
    
    global positionData;
    global positionTime;
    global frameRate;
    global angles; 
    % first time - generate an array and a plot
    if isempty(lastFrameTime)
        % initialize statics
        bufferModulo = 256;
        arrayIndex = 1;
        lastFrameTime = frameOfData.fLatency;
        lastFrameID = frameOfData.iFrame;
    end

    % calculate the frame increment based on mocap frame's timestamp
    % in general this should be monotonically increasing according
    % To the mocap framerate, however frames are not guaranteed delivery
    % so to be accurate we test and report frame drop or duplication
    newFrame = true;
    droppedFrames = false;
    frameTime = frameOfData.fLatency;
    
    frameID = frameOfData.iFrame;
    if(frameID ==lastFrameID)
        %debug
        %fprintf('same id\n');
    end

    calcFrameInc = round( (frameTime - lastFrameTime) * frameRate );
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
                
            if(frameOfData.RigidBodies.Length() > 0)
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
                

                angles = extractAngles( frameOfData.RigidBodies(1) );           
                
                x_off = frameOfData.RigidBodies(1).x;
                y_off = frameOfData.RigidBodies(1).y;
                z_off = frameOfData.RigidBodies(1).z;

%                 positionData = [frameOfData.RigidBodies(2).x-x_off, -(frameOfData.RigidBodies(2).z-z_off), frameOfData.RigidBodies(2).y-y_off, ...
%                                 frameOfData.RigidBodies(3).x-x_off, -(frameOfData.RigidBodies(3).z-z_off), frameOfData.RigidBodies(3).y-y_off, ...
%                                 frameOfData.RigidBodies(4).x-x_off, -(frameOfData.RigidBodies(4).z-z_off), frameOfData.RigidBodies(4).y-y_off, ...
%                                 frameOfData.RigidBodies(5).x-x_off, -(frameOfData.RigidBodies(5).z-z_off), frameOfData.RigidBodies(5).y-y_off];
               positionData = [x_off,y_off,z_off]; 
                positionTime = frameTime * frameRate;
                
           end
        end
    catch err
        display(err);
    end
    
    lastFrameTime = frameTime;
    lastFrameID = frameID;

end

function angles = extractAngles( rigidBody )

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