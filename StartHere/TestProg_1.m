function TestProg_1(serPort)
% Robot moves along a path, part blindly, part sensing.
% serPort is the serial port number (for controlling the actual robot).

%%%% DO NOT MODIFY CODE ABOVE %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp ('==================')
disp ('Program Starting  ')
disp ('------------------')


travelDist(serPort, .5, 2)  % Move .5m/s (max. speed) for +2m
          % Speed in [.025,.5] meters/s
          % Positive distance means move forward
          % Negative distance means move backward
          % During this command nothing else is checked... no good
          %   to test sensor inputs, i.e. not possible to avoid walls!

turnAngle(serPort, .2, 90)  % Turn at .2rad/s for +90 degrees
          % Speed in [.025,.2] radians/second
          % Positive angle means counter-clockwise
          % Negative angle means clockwise

travelDist(serPort, .5, 2.5)
turnAngle (serPort, .2, 90)

% Read sonar value, out of range is returned as not a result! Hence
%   needs checking. Using a real value larger than the map is one
%   option to work with it, but no the only or necessarily the best!
% sonar 1 is Left, 2 Front, 3 Right, 4 Rear.
% Son(ar)FF(front sensor) will save the last value read in meters.
SonFF = ReadSonar(serPort, 2)
if ~any(SonFF) SonFF = 100
end

%sets forward velocity using differential system
SetDriveWheelsCreate(serPort, 0.4, 0.4)

% Now check the sonar regularly while the robot moves,
% till the distance in front drops below a desired value...
while ( SonFF > 0.5)
    pause(.1)
    SonFF = ReadSonar(serPort, 2);
    if ~any(SonFF) SonFF = 100;
    end
end

disp (SonFF)

% stop the robot
SetDriveWheelsCreate(serPort, 0, 0)
