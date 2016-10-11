function Lab2(serPort)
% serPort is the serial port number (for controlling the actual robot).

disp ('===========================')
disp ('Lab2: Robot Initiated...   ')
disp ('------------')

% travelDist(serPort, .5, 2)
% %--
% turnAngle(serPort, .2, 90)
% %--^
% travelDist(serPort, .5, 3)
% %  |
% %  |
% %--|
% turnAngle (serPort, .2, -90)
% %  |>
% %  |
% %--|
% travelDist(serPort, .5, 1)
% %  |-
% %  |
% %--|
% turnAngle (serPort, .2, -90)
% %  |-v
% %  |
% %--|
% travelDist(serPort, .5, 3)
% %  |-|
% %  | |
% %--| |
% turnAngle (serPort, .2, 90)
% %  |-|
% %  | |
% %--| |>
% travelDist(serPort, .5, 1)
% %  |-|
% %  | |
% %--| |-
% turnAngle (serPort, .2, 90)
% %  |-|
% %  | |
% %--| |-^
% travelDist(serPort, .5, 3)
% %  |-| |
% %  | | |
% %--| |-|
% turnAngle (serPort, .2, -90)
% %  |-| |>
% %  | | |
% %--| |-|
% travelDist(serPort, .5, 1)
% %  |-| |-
% %  | | |
% %--| |-|
% turnAngle (serPort, .2, -90)
% %  |-| |-v
% %  | | |
% %--| |-|
% travelDist(serPort, .5, 3)
% %  |-| |-|
% %  | | | |
% %--| |-| |
% turnAngle (serPort, .2, 90)
% %  |-| |-|
% %  | | | |
% %--| |-| |>
% travelDist(serPort, .5, 1)
% %  |-| |-|
% %  | | | |
% %--| |-| |-
% turnAngle (serPort, .2, 90)
% %  |-| |-|
% %  | | | |
% %--| |-| |-^
% travelDist(serPort, .5, 3)
% %  |-| |-| |
% %  | | | | |
% %--| |-| |-|
% turnAngle (serPort, .2, -90)
% %  |-| |-| |>
% %  | | | | |
% %--| |-| |-|
% travelDist(serPort, .5, 1)
% %  |-| |-| |-
% %  | | | | |
% %--| |-| |-|
% turnAngle (serPort, .2, -90)
% %  |-| |-| |-v
% %  | | | | |
% %--| |-| |-|
% travelDist(serPort, .5, 4)
% %  |-| |-| |-|
% %  | | | | | |
% %--| |-| |-| |
% %            |
% turnAngle (serPort, .2, -90)
% %  |-| |-| |-|
% %  | | | | | |
% %--| |-| |-| |
% %           <|
% travelDist(serPort, .5, 7)
% %  |-| |-| |-|
% %  | | | | | |
% %--| |-| |-| |
% %-----------<|

%Using a for loop, we need to replicate the above commands
forward_commands = [2, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 4, 7];
turning_commands = [90, -90, -90, 90, 90, -90, -90, 90, 90, -90, -90, -90];

for i=1:13,
    travelDist(serPort, .5, forward_commands(i))
    if(i<13)
   turnAngle (serPort, .2, turning_commands(i))
    end
end
% stop the robot

SetDriveWheelsCreate(serPort, 0, 0)
