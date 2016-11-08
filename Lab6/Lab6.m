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
    
% Read the distance (odometry) sensor are initialize distance accumulator
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


wandering = 0;
homing = 0;
following = 1;
distance_to_left = 0.5;
distance_to_finish = 0.5;
e100 = zeros(100,1);

t = 0;
% Now check the Lidar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...
while(wandering || homing || following)
    pause(.1)

    t = t + 1;
    % Read Left side of Lidar
    LidarRes = LidarSensorCreate(serPort);
    [LidarM, LidarD] = min(LidarRes(341:681));
    
    
    if (following)
        
        %are we there yet?!
        if (Dist1 >= 10)
           following = 0;
           wandering = 0;
           homing = 0;
           continue
        end
        
        %Are we lost?! (checking if we are in the middle of the map, where no
        %walls are visible from the sonars)
%         if (SonFF(idx1) == 100 && SonRiF(idx1) == 100 && SonReF(idx1) == 100 && SonLF(idx1) == 100)
%            lost_counter = lost_counter + 1;
%            if(lost_counter > 5)
%                wandering = 1;
%                homing = 0;
%                following = 0;
%                continue
%            end
%         else
%             %reset lost_counter. we need 5 consecutive "lost" conditions.
%             lost_counter = 0;
%         end

        % Read Odometry sensor and accumulate distance measured.
        DistRead = DistanceSensorRoomba(serPort);
        Dist1 = Dist1 + DistRead;

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
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
    end 
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    