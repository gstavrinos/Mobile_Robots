function Lab4Prog_eg1(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp ('==================')
disp ('Program Starting  ')
disp ('------------------')

%  Use a command like this one to plot the trajectory followed from the
%  saved data (double-click in your saved data file to load it into the
%  datahistory workspace). Check the size and adjuts the index accordingly.
%
%  plot(cell2mat(datahistory(1:2602,2)),cell2mat(datahistory(1:2602,3)));


% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.5, 0.5);

% Initialise variables plus 1st sensor read for each.
    idx1 = 1;

% We need to check each sonar for valid reading in case is out of range.
    SonRead = ReadSonar(serPort, 3);
    if ~any(SonRead) SonLF(idx1) = 100;
    else SonLF(idx1) = SonRead;
    end
    
    SonRead = ReadSonar(serPort, 2);
    if ~any(SonRead) SonFF(idx1) = 100;
    else SonFF(idx1) = SonRead;
    end
    
    disp('Stating Task 1 - wandering');
    %------------------------------------------------------------------------------
    % Task-1: wander avoiding obstacles
    Camera = CameraSensorCreate(serPort);
    while (~any(Camera))
        
        [BumpRight,BumpLeft,WheDropRight,WheDropLeft,WheDropCaster,BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        if ~BumpRight && ~BumpLeft && ~BumpFront
            SetDriveWheelsCreate(serPort, 0.3, 0.3);
        elseif BumpRight
                turnAngle (serPort, .2, 70);
        elseif BumpLeft
                turnAngle (serPort, .2, -70);
        else
                turnAngle (serPort, .2, 100);
        end
            
        pause (.1);
        Camera = CameraSensorCreate(serPort);
        
    end
    
    disp('Saw beacon! Stating Task 2 - homing');
    %------------------------------------------------------------------------------
    % Task-2: homing into (reach) the beacon
    while (any(Camera))
        Camera = CameraSensorCreate(serPort);
        if any(Camera) && abs(Camera) > 0.05
                turnAngle (serPort, .2, (Camera * 6));
        end
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
        pause (.1);
    end

    disp('Passed the beacon. Stating Task 3 - following');
    %------------------------------------------------------------------------------
    % Task-3: check the sonar regularly while the robot moves, to keep the distance
    % to the wall in the desired range, and avoiding colliding with a wall both
    % on the side and in front. Repeat for a given distance...
    
    % Read the distance (odometry) sensor are initialize distance accumulator
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = 0;

    while (Dist1 < 19)
        pause(.1)

        % Read Left Sonar and correct if out-of-range
        SonRead = ReadSonar(serPort, 3);
        if ~any(SonRead) SonLF(idx1) = 100;
        else SonLF(idx1) = SonRead;
        end
    
        % If distance out of limits turn to get near/away from the wall
        if ( SonLF(idx1) > 0.31 ) turnAngle(serPort, .2, 5);
        elseif ( SonLF(idx1) < 0.28 ) turnAngle(serPort, .2, -5);
        end

        % Read Front Sonar and correct if out-of-range
        SonRead = ReadSonar(serPort, 2);
        if ~any(SonRead) SonFF(idx1) = 100;
        else SonFF(idx1) = SonRead;
        end
    
        % It too near a wall it means needs to turn sharp right!
        if ( SonFF(idx1) < 0.30 ) turnAngle(serPort, .2, -70);
        end
    
        % Read Odometry sensor and accumulate distance measured.
        DistRead = DistanceSensorRoomba(serPort);
        Dist1 = Dist1 + DistRead;
    
        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
    end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0)

    % Display last Left sonar reading and total distance travelled
    disp (SonLF(idx1))
    disp (Dist1)
    