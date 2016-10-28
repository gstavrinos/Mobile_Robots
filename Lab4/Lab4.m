function Lab4(serPort)

disp ('====================')
disp ('Lab4 Robot Initiated')
disp ('--------------------')

% Initialise variables plus 1st sensor read for each.
idx1 = 1;

wandering = 1;
homing = 0;
following = 0;
distance_to_left = 0.5;
distance_to_finish = 0.5;

lost_counter = 0;

disp('Starting Task 1 - wandering');

%Let's preallocate memory for each sonar read
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
end

SonRead = ReadSonar(serPort, 4);
if ~any(SonRead) 
    SonReF(idx1) = 100;
else
    SonReF(idx1) = SonRead;
end

while(wandering || homing || following)
    
    pause(.1);
    
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
    end
    
    SonRead = ReadSonar(serPort, 4);
    if ~any(SonRead) 
        SonReF(idx1) = 100;
    else
        SonReF(idx1) = SonRead;
    end
    
    %------------------------------------------------------------------------------
    % Task-1: wander avoiding obstacles
    if(wandering)
        
        if (SonFF(idx1) < 0.5)
            if ( SonLF(idx1) < SonRiF(idx1)) 
                turnAngle(serPort, .2, -40);
            else
                turnAngle(serPort, .2, 40);
            end
        else
            if ( SonLF(idx1) < SonRiF(idx1) && SonLF(idx1) < 0.5) 
                turnAngle(serPort, .2, -10);
            elseif (SonLF(idx1) > SonRiF(idx1) && SonRiF(idx1) < 0.5)
                turnAngle(serPort, .2, 10);
            end
        end

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
        Camera = CameraSensorCreate(serPort);
        if(any(Camera))
           wandering = 0;
           homing = 1;
           following = 0;
           disp('Saw beacon! Starting Task 2 - homing');
        end
    end
    
    %------------------------------------------------------------------------------
    % Task-2: homing into (reach) the beacon
    if (homing)
        Camera = CameraSensorCreate(serPort);
        if any(Camera) && abs(Camera) > 0.05
                turnAngle (serPort, .2, (Camera * 6));
        end
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
        
        if(~any(Camera))
            wandering = 0;
            homing = 0;
            following = 1;
            disp('Passed the beacon. Starting Task 3 - following');
            Dist1 = 0;
            lost_counter = 0;
            distance_to_finish = SonFF(idx1);
            distance_to_left = distance_to_finish;
        end
    end

    %------------------------------------------------------------------------------
    % Task-3: check the sonar regularly while the robot moves, to keep the distance
    % to the wall in the desired range, and avoiding colliding with a wall both
    % on the side and in front. Repeat for a given distance...
    
    % Read the distance (odometry) sensor are initialize distance accumulator
   

    if (following)
        
        if (SonFF(idx1) < distance_to_finish && SonReF(idx1) > 1.5  && SonReF(idx1) < 3 && SonRiF(idx1) == 100 && Dist1 > 10)
           following = 0;
           wandering = 0;
           homing = 0;
           continue
        end
        
        %Are we lost?! (checking if we are in the middle of the map, where no
        %walls are visible from the sonars)
        if (SonFF(idx1) == 100 && SonRiF(idx1) == 100 && SonReF(idx1) == 100 && SonLF(idx1) == 100)
           lost_counter = lost_counter + 1;
           if(lost_counter > 5)
               wandering = 1;
               homing = 0;
               following = 0;
               continue
           end
        else
            %reset lost_counter. we need 5 consecutive "lost" conditions.
            lost_counter = 0;
        end

        if ( SonLF(idx1) > distance_to_left+.03 ) 
            turnAngle(serPort, .2, 4);
        elseif ( SonLF(idx1) > distance_to_left+.02 ) 
            turnAngle(serPort, .2, 3);
        elseif ( SonLF(idx1) < distance_to_left-.03 ) 
            turnAngle(serPort, .2, -4);
        elseif ( SonLF(idx1) < distance_to_left-.02 ) 
            turnAngle(serPort, .2, -3);
        end

        % If too near a wall it means needs to turn sharp right!
        if ( SonFF(idx1) <= distance_to_left ) 
            turnAngle(serPort, .2, -90);
        end

        % Read Odometry sensor and accumulate distance measured.
        DistRead = DistanceSensorRoomba(serPort);
        Dist1 = Dist1 + DistRead;

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
    end
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0)

    % Display last Left sonar reading and total distance travelled
    disp (SonLF(idx1))
    disp (Dist1)
    