function Lab9(serPort)

    disp ('====================')
    disp ('Lab9 Robot Initiated')
    disp ('--------------------')
    
    % Codes for each map region are in the following format:
    % AB for position and C for orientation where 
    % A is the first letter of the colour (R, B, G, Y)
    % B is the specific colours letter (A, B, C, D, E, F)
    % and C is a basic direction (U, D, L, R)
    
    % Let's define some connections on the map
    connections = { 'BA' 'RA';
                    'RA' 'BA';
                    'GA' 'RA';
                    'RA' 'GA';
                    'BB' 'GA';
                    'RB' 'BB';
                    'BG' 'RB';
                    'RC' 'BC';
                    'BC' 'RC';
                    'RD' 'BD';
                    'BD' 'RC';
                    'RE' 'BE';
                    'BE' 'RE';
                    'RE' 'BF';
                    'BF' 'RB';
                    'RB' 'GB';
                    'RC' 'GC';
                    'RD' 'GD'
                    };
                
    % Current estimated position
    currPos = 'UNKNOWN';
    
    % Current estimated orientation
    currOrient = 'UNKNOWN';
    
    % Check if wandering is still trying to approach a wall
    wandering_init = 1;
    
    last_pos_estimation = 'UNKNOWN';
    last_orient_estimation = 'UNKNOWN';
    
    % RED: B, C, D
    % Total = 2
    
    % RED: A
    % Total > 1.5 && Total < 2
    
    % RED: E
    % Total = 3

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
    [LidarL, LidarD] = min(LidarRes(341:681));
    
    % Check front ~45 degrees
    [LidarMmid, LidarDmid] = min(LidarRes(216:466));

    % This our plan. The sequence of tasks.
    plan = ['w', 'h', 'f'];
    
    % Obstacle Avoidance is disabled for now
    obstacle_avoidance = 0;

    % Initialize the Integral buffer
    e100 = zeros(100,1);

    % Iteration initialized to zero. It is used as a timer.
    t = 0;
    
    % Lost counter initialized to zero. It is used to get back to wandering
    % mode.
    lost_counter = 0;

    % Main loop
    while(size(plan) > 0)

        pause(.1)

        % Reset obstacle avoidance
        obstacle_avoidance = 0;
        
        % Increase timer
        t = t + 1;

        LidarRes = LidarSensorCreate(serPort);
        
        % Read odometry sensor
        % Give some extra time for the odom reading. Without this extra
        % pause, the odometry reading does not seem to return anything!
        pause(.1)
        DistRead = DistanceSensorRoomba(serPort);

        % Read Left side of Lidar
        [LidarL, LidarD] = min(LidarRes(341:681));

        % Read Right side of Lidar
        [LidarR, LidarD2] = min(LidarRes(1:341));
        
        
        % Check front ~45 degrees
        [LidarMmid, LidarDmid] = min(LidarRes(216:466));
        
        Camera = CameraSensorCreate(serPort);

        SonRead = ReadSonar(serPort, 4);
        if ~any(SonRead) 
            SonReF(1) = 100;
        else
            SonReF(1) = SonRead;
        end
        
        % Check if we need to avoid obstacles
        % Run the obstacle avoidance reactive task if needed
        obstacleAvoidance
        
        % Don't change anything on the planned tasks if we had to run a
        % reactive task in this loop
        if ~obstacle_avoidance
            % Change the plan based on our new environment
            planManager
            
            % Run the corresponding planned task
            if size(plan) > 0
                % Task-1: Wandering mode
                if plan(1) == 'w'
                    robotWandering
                % Task-2: Homing mode
                elseif plan(1) == 'h'
                    robotHoming
                % Task-3: Following mode
                elseif plan(1) == 'f'
                    robotFollowing
                end
            end
        end
    end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('I reached my goal! :)');

    
    % ------------------------------------------------------------------
    
    
    % Helper functions
    
    % Planned Task - Wandering around an unknown area.
    % This could also be a reactive task, but for me, is part of the plan,
    % since the robot does not know its initial position, so it should
    % start wandering until it finds the beacon.
    function robotWandering
%         % If there is not enough space in front of the robot
%         if (LidarRes(341) < 1.0)
%             if ( LidarL < LidarR) 
%                 turnAngle(serPort, .2, -40);
%             else
%                 turnAngle(serPort, .2, 40);
%             end
%         end
%         % Check the sides using the LIDAR and try to stay away from walls
%         if ( LidarL < LidarR && LidarL < 0.5) 
%             turnAngle(serPort, .2, -10);
%         elseif (LidarR > LidarR && LidarR < 0.5)
%             turnAngle(serPort, .2, 10);
%         end

        if ~wandering_init
            turnAngle(serPort, .2, PID);
        else
            if LidarMmid <= 0.6
                wandering_init = 0;
            end
        end

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        localization
    end

    % Planned Task - Homing to beacon
    function robotHoming
        % The simple homing task (without obstacle avoidance) is used as
        % it was provided.
        if (any(Camera) && abs(Camera) > 0.05)
            turnAngle (serPort, .2, (Camera * 6));
        end 
        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.4, 0.4);
    end

    % Planned Task - Following the path
    function robotFollowing

        % Obey your PID controller make a turn that equals P + I + D
        turnAngle(serPort, .2, PID);

        % Accumulate distance measured.
        Dist1 = Dist1 + DistRead;

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
    end

    % This is a method that manages the different planned tasks. This could
    % be described as a reactive task, although it is related to the
    % requirements of the plan.
    function planManager
        if plan(1) == 'w'
            % Check the camera for the beacon
            if(any(Camera))
               plan = ['h', 'f'];
               disp('Saw beacon! Starting Task 2 - homing');
            end
        elseif plan(1) == 'h'
            if(~any(Camera))
                plan = 'f';
                disp('Passed the beacon. Starting Task 3 - following');
                Dist1 = 0;
                lost_counter = 0;
            end
        elseif plan(1) == 'f'
            % Are we there yet?!
            if (Dist1 >= 17 && SonReF(1) > 2.1 && SonReF(1) < 100)
               plan = [];
            end

            % Are we lost?! (checking if left side is >= 1m for 5 
            % consecutive measurements)
            if (LidarL >= 1.0)
               lost_counter = lost_counter + 1;
               if(lost_counter > 5)
                   plan = ['w', 'h', 'f'];
                   disp('I am lost! Starting Task 1 - wandering');
               end
            else
                % Reset lost_counter. We need 5 consecutive "lost" conditions.
                lost_counter = 0;
            end
        end
    end

    % Reactive task
    function obstacleAvoidance
        % Try to avoid the obstacles
        if (LidarMmid < 0.3)
            obstacle_avoidance  = 1;
            turnAngle (serPort, .2, (LidarDmid-341)/4);
        end
    end



    % The PID controller implementation
    function O = PID
        % Now do a simple porportional action. This part is used as was
        % provided, but with a change on the variable names.
        et = LidarL - 0.5;
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
    end

    % A function that tries to imitate localization
    function localization
%         % Read Left side of Lidar (216:466)
%         [LidarL2, LidarD] = min(LidarRes(501:521));
% 
%         % Read Right side of Lidar
%         [LidarR2, LidarD2] = min(LidarRes(161:181));

     SonRead = ReadSonar(serPort, 1);
        if ~any(SonRead) 
            LidarR2(1) = 100;
        else
            LidarR2(1) = SonRead;
        end

        SonRead = ReadSonar(serPort, 2);
        if ~any(SonRead) 
            LidarMmid(1) = 100;
        else
            LidarMmid(1) = SonRead;
        end

        SonRead = ReadSonar(serPort, 3);
        if ~any(SonRead)
            LidarL2(1) = 100;
        else
            LidarL2(1) = SonRead;
        end
        
        LidarL2
        LidarR2
        
        estimatedPos = {'UNKNOWN'};
        estimatedOrient = {'UNKNOWN'};
        blue_chaos = 0;
        if approx(LidarL2, 0.5) && LidarMmid >= 0.7
            % Check if we are in a Blue region
            if LidarR2 > 3
               estimatedPos = {'BA' 'BB' 'BC' 'BD' 'BE' 'BF'};
               estimatedOrient = {'U' 'D' 'L' 'L' 'L' 'U'};
               blue_chaos = 1;
            % Check if we are in Red E
            elseif approx(LidarR2, 2.5)
                estimatedPos = {'RE'};
                if LidarMmid <= 3 && LidarR2 > 1.2
                    estimatedOrient = {'D'};
                else
                    estimatedOrient = {'U'}; 
                end
            % Check if we are in one of the rest Red regions
            elseif approx(LidarR2, 1.5)
                if LidarMmid > 3
                %TODO I can't think of an orientation characteristic, yet!
                   % TODO I need a way to distinguish the two Bs and two As!
                   estimatedPos = {'RA' 'RB' 'RC' 'RD'}; 
                   estimatedOrient = {'UNKNOWN' 'U' 'U'};
                else
                    disp('RC/RD')
                   estimatedPos = {'RC' 'RD'}; 
                   estimatedOrient = {'D' 'D'};
                end
            end
        end
        filtered_pos_estimations = {};
        filtered_orient_estimations = {};
        if ~strcmp(currPos,'UNKNOWN')
            for i=1:numel(estimatedPos)
                for j=1:size(connections,1)
                   % With this OR we make sure that each region
                   % connects with itself, and regions are interconnected!
                   if strcmp(connections(j,2),estimatedPos(i)) && strcmp(connections(j,1),currPos)
                        filtered_pos_estimations(end+1) = estimatedPos(i);
                        filtered_orient_estimations(end+1) = estimatedOrient(i);
                   end
                end
            end
            if numel(filtered_pos_estimations) > 0
                disp('found path!')
                filtered_pos_estimations(1)
                if numel(filtered_pos_estimations) == 1
                    currPos = filtered_pos_estimations(1);
                    currOrient = filtered_orient_estimations(1);
                else
                    for i=1:numel(filtered_pos_estimations)
                        % Check some multiple paths that change based on
                        % orientation only
                       if ((strcmp(currPos,'RE') && strcmp(currOrient,'D') && strcmp(filtered_pos_estimations(i),'BE'))||(strcmp(currPos,'RE') && strcmp(currOrient,'U') && strcmp(filtered_pos_estimations(i),'BF')))
                            currPos = filtered_pos_estimations(i);
                            currOrient = filtered_orient_estimations(i);
                            break;
                       end
                    end
                end
                if ~strcmp(currPos,'UNKNOWN')
                    last_pos_estimation = currPos;
                    last_orient_estimation = currOrient;
                end
                
            elseif numel(estimatedPos) == 1
                if strcmp(estimatedPos(1),last_pos_estimation)
                    currPos = estimatedPos(1);
                    currOrient = estimatedOrient(1);
                else
                    if ~strcmp(estimatedPos(i),'UNKNOWN')
                        last_pos_estimation = estimatedPos(1);
                        last_orient_estimation = estimatedOrient(1);
                    end
                end
            end
        else
            if numel(estimatedPos) == 1 && ~strcmp(estimatedPos(1),'UNKNOWN')
                disp('I used to be lost, man...')
                currPos = estimatedPos(1);
                currOrient = estimatedOrient(1);
                last_pos_estimation = estimatedPos(1);
                last_orient_estimation = estimatedOrient(1);
            elseif blue_chaos
                disp('I know that I am in a blue region, but in which?')
            end
        end
        currPos
        currOrient
    end

    % A helper function to approximate values
    function result = approx(number1, number2)
        result = 0;
        if abs(number1 - number2) <= 0.2
            result = 1;
        end
    end
end
    