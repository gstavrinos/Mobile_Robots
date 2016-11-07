function Lab6Prog_eg1(serPort)
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

% Initialise variables plus 1st seonr read for each.
    idx1 = 1;
    
    % Proportional Gain
    KP = 100;
    
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

    
% Now check the Lidar regularly while the robot moves, to keep the distance
% to the wall in the desired range, and avoiding colliding with a wall both
% on the side and in front.
% Repeat for a given distance...
while (Dist1 < 18)
    pause(.1)

    % Read Left side of Lidar
    LidarRes = LidarSensorCreate(serPort);
    [LidarM, LidarD] = min(LidarRes(341:681));
    
    % Now do a simple porportional action, turning proportionally to how
    % far is from the set-point (0.5)
    % Watch the sign of the error!!
    DError = LidarM - 0.5;
    KOut = KP * DError;
    % We also need to limit the action, in this case max correction to
    % 50Deg (or -50 but that one is automatically limmited by reading 0)
    if (KOut > 50) KOut = 50;
    end
    turnAngle(serPort, .2, KOut);
    
    % Read Odometry sensor and accumulate distance measured.
    DistRead = DistanceSensorRoomba(serPort);
    Dist1 = Dist1 + DistRead;
    
    % Reset straight line and advance
    SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    