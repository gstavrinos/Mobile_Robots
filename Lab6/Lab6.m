function Lab6(serPort)

disp ('====================')
disp ('Lab6 Robot Initiated')
disp ('--------------------')

%  Use a command like this one to plot the trajectory followed from the
%  saved data (double-click in your saved data file to load it into the
%  datahistory workspace). Check the size and adjuts the index accordingly.
%
%  plot(cell2mat(datahistory(1:2602,2)),cell2mat(datahistory(1:2602,3)));


% Sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.5, 0.5);
    
% Proportional Gain
Kp = 100;
Ki = 0.0001;
Kd = 100;
    
% Read the distance (odometry) sensor and initialize distance accumulator
DistRead = DistanceSensorRoomba(serPort);
Dist1 = 0;

% Read the Lidar. It returns a vector of 680 values, spread symmetrically
% for 120 degrees each side of the front.
% Mid-range (element 341) is the front.
% Values above that element measure the Left side, and below that element
% measure the right side.
% To follow the line on the left, we want to get the minimum value to the
% left, i.e. in elements 341-681. 
LidarRes = LidarSensorCreate(serPort);
[LidarM, LidarD] = min(LidarRes(341:681));


wandering = 1;
homing = 0;
following = 0;
distance_to_left = 0.5;
distance_to_finish = 0.5;
e100 = zeros(100,1);

t = 0;
% Now check the Lidar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...

disp('I dont know my initial position! Starting Task 1 - wandering');

while(wandering || homing || following)
    
    pause(.1)

    t = t + 1;
    % Read Left side of Lidar
    LidarRes = LidarSensorCreate(serPort);
    [LidarM, LidarD] = min(LidarRes(341:681));
    [LidarM2, LidarD2] = min(LidarRes(1:341));

    
    %------------------------------------------------------------------------------
    % Task-1: wander avoiding obstacles
    if(wandering)
        
        if (LidarRes(341) < 1.0)
            if ( LidarM < LidarM2) 
                turnAngle(serPort, .2, -40);
            else
                turnAngle(serPort, .2, 40);
            end
        else
            if ( LidarM < LidarM2 && LidarM < 0.5) 
                turnAngle(serPort, .2, -10);
            elseif (LidarM2 > LidarM2 && LidarM2 < 0.5)
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
        %check front ~45 degrees
        [LidarMmid, LidarDmid] = min(LidarRes(216:466));
        Camera = CameraSensorCreate(serPort);
        if (any(Camera) && abs(Camera) > 0.05 && LidarMmid >= 0.3)
            turnAngle (serPort, .2, (Camera * 6));
        else
            if (LidarMmid < 0.3)
                turnAngle (serPort, .2, (LidarDmid-341)/4);
            elseif(~any(Camera))
                wandering = 0;
                homing = 0;
                following = 1;
                disp('Passed the beacon. Starting Task 3 - following');
                Dist1 = 0;
                lost_counter = 0;
                %distance_to_finish = SonFF(idx1);
                %distance_to_left = distance_to_finish;
            end
            
        end
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
        
        
    end
    
    
    %------------------------------------------------------------------------------
    % Task-3: check the sonar regularly while the robot moves, to keep the distance
    % to the wall in the desired range, and avoiding colliding with a wall both
    % on the side and in front. Repeat for a given distance...
    
    % Read the distance (odometry) sensor are initialize distance accumulator
    if (following)
        
        %are we there yet?!
        if (Dist1 >= 10)
           following = 0;
           wandering = 0;
           homing = 0;
           disp('I think I reached my goal! :)');
           continue
        end
        
        %Are we lost?! (checking left side is >= 1m for 5 consecutive measurements)
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
            %reset lost_counter. we need 5 consecutive "lost" conditions.
            lost_counter = 0;
        end
        
        % Now do a simple porportional action, turning proportionally to how
        % far is from the set-point (0.5)
        % Watch the sign of the error!!
        et = LidarM - 0.5;
        P = Kp * et;
        % We also need to limit the action, in this case max correction to
        % 50Deg (or -50 but that one is automatically limmited by reading 0)
        if (P > 50) 
            P = 50;
        end
        e100(mod(t,100)+1) = et;
        I = Ki * sum(e100);
        
        if t < 100
            D = Kd * ((e100(t)-e100(1))/t);
        else
            D = Kd * ((e100(mod(t,100)+1)-e100(mod(t-99,100)+1))/100);
        end;
        
        O = P + I + D;
        
        turnAngle(serPort, .2, O);
        
        % Read Odometry sensor and accumulate distance measured.
        DistRead = DistanceSensorRoomba(serPort);
        Dist1 = Dist1 + DistRead;

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        %Give some extra time for the odom reading
        pause(.1)
    end 
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    