function getPositionData( frameOfData )

    persistent lastFrameTime;
    persistent lastFrameID;
    persistent arrayIndex;
    persistent bufferModulo;
    
    global positionData;
    global positionTime;
    global frameRate;
    
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
                
                x_off = frameOfData.RigidBodies(1).x;
                y_off = frameOfData.RigidBodies(1).y;
                z_off = frameOfData.RigidBodies(1).z;

                positionData = [frameOfData.RigidBodies(2).x-x_off, -(frameOfData.RigidBodies(2).z-z_off), frameOfData.RigidBodies(2).y-y_off, ...
                                frameOfData.RigidBodies(3).x-x_off, -(frameOfData.RigidBodies(3).z-z_off), frameOfData.RigidBodies(3).y-y_off, ...
                                frameOfData.RigidBodies(4).x-x_off, -(frameOfData.RigidBodies(4).z-z_off), frameOfData.RigidBodies(4).y-y_off, ...
                                frameOfData.RigidBodies(5).x-x_off, -(frameOfData.RigidBodies(5).z-z_off), frameOfData.RigidBodies(5).y-y_off];
                
                positionTime = frameTime * frameRate;
                
           end
        end
    catch err
        display(err);
    end
    
    lastFrameTime = frameTime;
    lastFrameID = frameID;

end

