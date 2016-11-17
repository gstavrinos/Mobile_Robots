function Lab6(serPort)

disp ('====================')
disp ('Lab6 Robot Initiated')
disp ('--------------------')

% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.5, 0.5);
    
% Proportional Gain
Kp = 200;
% Integral Gain
Ki = 0.001;
% Differential Gain
Kd = 150;
    
% Read the distance (odometry) sensor and initialize distance accumulator
DistRead = DistanceSensorRoomba(serPort);
Dist1 = 0;

% Read the back sonar so that we know when to stop!
SonRead = ReadSonar(serPort, 4);
if ~any(SonRead) 
    SonReF(1) = 100;
else
    SonReF(1) = SonRead;
end

% Read the Lidar. It returns a vector of 680 values, spread symmetrically
% for 120 degrees each side of the front.
% Mid-range (element 341) is the front.
% Values above that element measure the Left side, and below that element
% measure the right side.
LidarRes = LidarSensorCreate(serPort);
[LidarM, LidarD] = min(LidarRes(341:681));


%set the FSM's states
wandering = 1;
homing = 0;
following = 0;

%initialize the Integral buffer
e100 = zeros(100,1);

%iteration initialized to zero. It is used as a timer.
t = 0;

disp('I dont know my initial position! Starting Task 1 - wandering');

while(wandering || homing || following)
    
    pause(.1)

    % Increase timer
    t = t + 1;
    
    LidarRes = LidarSensorCreate(serPort);
    
    % Read Left side of Lidar
    [LidarM, LidarD] = min(LidarRes(341:681));
    
    % Read Right side of Lidar
    [LidarM2, LidarD2] = min(LidarRes(1:341));
    
    SonRead = ReadSonar(serPort, 4);
    if ~any(SonRead) 
        SonReF(1) = 100;
    else
        SonReF(1) = SonRead;
    end

    
    %------------------------------------------------------------------------------
    % Task-1: Wandering mode
    if(wandering)
        
        % If there is not enough space in front of the robot
        if (LidarRes(341) < 1.0)
            if ( LidarM < LidarM2) 
                turnAngle(serPort, .2, -40);
            else
                turnAngle(serPort, .2, 40);
            end
        else
            % Check the sides using the LIDAR and try to stay away from
            % obstacles and walls
            if ( LidarM < LidarM2 && LidarM < 0.5) 
                turnAngle(serPort, .2, -10);
            elseif (LidarM2 > LidarM2 && LidarM2 < 0.5)
                turnAngle(serPort, .2, 10);
            end
        end

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
        Camera = CameraSensorCreate(serPort);
        % Check the camera for the beacon
        if(any(Camera))
           wandering = 0;
           homing = 1;
           following = 0;
           disp('Saw beacon! Starting Task 2 - homing');
        end
    end
    
    
    %------------------------------------------------------------------------------
    % Task-2: Homing mode
    if (homing)
        % Check front ~45 degrees
        [LidarMmid, LidarDmid] = min(LidarRes(216:466));
        Camera = CameraSensorCreate(serPort);
        % The simeple homing task (without obstacle avoidance) is used as
        % it was provided.
        if (any(Camera) && abs(Camera) > 0.05 && LidarMmid >= 0.3)
            turnAngle (serPort, .2, (Camera * 6));
        else
            % Try to avoid the obstacles between the robot and the beacon
            if (LidarMmid < 0.3)
                turnAngle (serPort, .2, (LidarDmid-341)/4);
            elseif(~any(Camera))
                wandering = 0;
                homing = 0;
                following = 1;
                disp('Passed the beacon. Starting Task 3 - following');
                Dist1 = 0;
                lost_counter = 0;
            end
            
        end
        
        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
        
    end
    
    
    %------------------------------------------------------------------------------
    % Task-3: Following mode
    if (following)
        
        % Are we there yet?!
        if (Dist1 >= 17 && SonReF(1) > 2.1 && SonReF(1) < 100)
           following = 0;
           wandering = 0;
           homing = 0;
           disp('I reached my goal! :)');
           continue
        end
        
        % Are we lost?! (checking if left side is >= 1m for 5 
        % consecutive measurements)
        if (LidarM >= 1.0)
           lost_counter = lost_counter + 1;
           if(lost_counter > 5)
               wandering = 1;
               homing = 0;
               following = 0;
               disp('I am lost! Starting Task 1 - wandering');
               continue
           end
        else
            % Reset lost_counter. We need 5 consecutive "lost" conditions.
            lost_counter = 0;
        end
        
        % Now do a simple porportional action. This part is used as was
        % provided, but with a change on the variable names.
        et = LidarM - 0.5;
        P = Kp * et;
        if (P > 50) 
            P = 50;
        end
        % Integral part. Push the new error to the buffer, and calculate
        % the new I.
        e100(mod(t,100)+1) = et;
        I = Ki * sum(e100);
        
        % Differential part. Calculate D based on the last 100 errors (if
        % we have that many)
        if t < 100
            D = Kd * ((e100(t)-e100(1))/t);
        else
            D = Kd * ((e100(mod(t,100)+1)-e100(mod(t-99,100)+1))/100);
        end;
        
        % Final PID controller's decision
        O = P + I + D;
        
        % Obey your PID controller make a turn that equals P + I + D
        turnAngle(serPort, .2, O);
        
        % Read Odometry sensor and accumulate distance measured.
        DistRead = DistanceSensorRoomba(serPort);
        Dist1 = Dist1 + DistRead;

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
        % Give some extra time for the odom reading
        pause(.1)
    end 
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    