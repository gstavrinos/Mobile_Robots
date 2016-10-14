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
%The commands below are the ones that we should run if the motor encoders
%had 100% accuracy
forward_commands = [2, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 4, 7];
turning_commands = [90, -90, -90, 90, 90, -90, -90, 90, 90, -90, -90, -90];

%By uncommenting the commands below, we get very close to the desired path
%forward_commands = [1.8, 2.7, 0.7, 2.7, 0.65, 2.7, 0.65, 2.7, 0.65, 2.7, 0.65, 3.7, 6.8];
%turning_commands = [84, -84, -84, 84, 84, -84, -84, 84, 84, -84, -85, -85];

for i=1:13,
    travelDist(serPort, .5, forward_commands(i))
    if(i<13)
   turnAngle (serPort, .2, turning_commands(i))
    end
end
% stop the robot

SetDriveWheelsCreate(serPort, 0, 0)
