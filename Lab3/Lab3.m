function Lab3(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp ('==================')
disp ('Program Starting  ')
disp ('------------------')

% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.5, 0.5);

% Initialise variables plus 1st seonr read for each.
    idx1 = 1;

% We need to check each sonar for valid reading in case is out of range.
    distance_to_left = 0.5;
    distance_to_finish = 0.5;

    SonRead = ReadSonar(serPort, 1);
    if ~any(SonRead) 
        SonRiF(idx1) = 100;
    else
        SonRiF(idx1) = SonRead;
    end
    
    SonRead = ReadSonar(serPort, 2);
    if ~any(SonRead) 
        SonFF(idx1) = 100;
    else
        SonFF(idx1) = SonRead;
    end
    
    SonRead = ReadSonar(serPort, 3);
    if ~any(SonRead)
        SonLF(idx1) = 100;
    else
        SonLF(idx1) = SonRead;
        distance_to_left = SonRead;
    end
    
    
    SonRead = ReadSonar(serPort, 4);
    if ~any(SonRead) 
        SonReF(idx1) = 100;
    else
        SonReF(idx1) = SonRead;
        distance_to_finish = SonRead;
    end
    
% Read the distance (odometry) sensor are initialize distance accumulator
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = 0;

% Now check the sonar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...
while (1)
    pause(.1)

    % Read Right Sonar and correct if out-of-range
    SonRead = ReadSonar(serPort, 1);
    if ~any(SonRead) 
        SonRiF(idx1) = 100;
    else
        SonRiF(idx1) = SonRead;
    end

    % Read Front Sonar and correct if out-of-range
    SonRead = ReadSonar(serPort, 2);
    if ~any(SonRead) 
        SonFF(idx1) = 100;
    else
        SonFF(idx1) = SonRead;
    end
    
    % Read Left Sonar and correct if out-of-range
    SonRead = ReadSonar(serPort, 3);
    if ~any(SonRead) 
        SonLF(idx1) = 100;
    else
        SonLF(idx1) = SonRead;
    end
    
    % Read Rear Sonar and correct if out-of-range
    SonRead = ReadSonar(serPort, 4);
    if ~any(SonRead) 
        SonReF(idx1) = 100;
    else
        SonReF(idx1) = SonRead;
    end
    
    % If distance out of limits turn to get near/away from the wall
    if ( SonLF(idx1) > distance_to_left+.03 ) 
        turnAngle(serPort, .2, 4);
    elseif ( SonLF(idx1) > distance_to_left+.02 ) 
        turnAngle(serPort, .2, 3);
    %elseif ( SonLF(idx1) > distance_to_left+.02 ) 
    %    turnAngle(serPort, .2, 1);
    elseif ( SonLF(idx1) < distance_to_left-.03 ) 
        turnAngle(serPort, .2, -4);
    elseif ( SonLF(idx1) < distance_to_left-.02 ) 
        turnAngle(serPort, .2, -3);
    %elseif ( SonLF(idx1) < distance_to_left-.02 ) 
    %    turnAngle(serPort, .2, -1);
    end
    
    % It too near a wall it means needs to turn sharp right!
    if ( SonFF(idx1) <= distance_to_left ) 
        turnAngle(serPort, .2, -90);
    end
    
    % Read Odometry sensor and accumulate distance measured.
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = Dist1 + DistRead;
    
    % Reset straight line and advance
    SetDriveWheelsCreate(serPort, 0.5, 0.5);
    
%     if(SonLF(idx1) > distance_to_left)
%         SetDriveWheelsCreate(serPort, 0.25, 0.1);
%     elseif (SonLF(idx1) < distance_to_left)
%         SetDriveWheelsCreate(serPort, 0.1, 0.25);
%     else
%         SetDriveWheelsCreate(serPort, 0.5, 0.5);
%     end
    
    if (SonFF(idx1) < distance_to_finish && SonReF(idx1) > 1.5  && SonReF(idx1) < 3 && SonRiF(idx1) == 100 && Dist1 > 10)
       break; %finish! 
    end
    
        
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0)

    % Display last Left sonar reading and total distance travelled
    disp (SonLF(idx1))
    disp (Dist1)
    
    load('path2.mat')
    figure
    plot(cell2mat(datahistory(:,2)),cell2mat(datahistory(:,3)));
    