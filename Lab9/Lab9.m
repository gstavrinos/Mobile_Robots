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
                    'GA' 'RB';
                    'GB' 'RC';
                    'GC' 'RD';
                    'GD' 'RE';
                    'BB' 'RA';
                    'RB' 'BB';
                    'BG' 'RB';
                    'RC' 'BC';
                    'BC' 'RC';
                    'RD' 'BD';
                    'BD' 'RD';
                    'RE' 'BE';
                    'BE' 'RE';
                    'RE' 'BF';
                    'BF' 'RB';
                    'RB' 'GB';
                    'RA' 'GA';
                    'RC' 'GC';
                    'RD' 'GD';
                    };
                
    % Current estimated position
    currPos = 'UNKNOWN';
    
    % Current estimated orientation
    currOrient = 'UNKNOWN';
    
    % Check if wandering is still trying to approach a wall
    wandering_init = 1;
    
    % We will write ALL localization entries to a file, in order to
    % evaluate the procedure easily
    t = datetime('now','TimeZone','local','Format','d-MMM-y_HH_mm_ss');
    folder = 'localization_sessions/';
    file = fopen(strcat(folder,strcat(char(t),'_localization_session.txt')),'w');
    
    % Avoid spamming the session files with the same values
    last_written_pos = 'NONE';
    last_written_orient = 'NONE';
    
    % We will use a kind of memory, to compare the last estimation with the
    % current estimation
    last_pos_estimation = 'UNKNOWN';
    last_orient_estimation = 'UNKNOWN';
    
    % I am going to use this variable to detect the major region changes by
    % detecting if the PID controller is sending a major turning command.
    heavy_turning = 0;

    % Sets forward velocity using differential system
    SetDriveWheelsCreate(serPort, 0.5, 0.5);

    % Proportional Gain
    Kp = 200;
    % Integral Gain
    Ki = 0.001;
    % Differential Gain
    Kd = 150;

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

    % This is our plan. The sequence of tasks.
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
    fclose(file);

    
    % ------------------------------------------------------------------
    
    
    % Helper functions
    
    % Planned Task - Wandering around an unknown area.
    % This could also be a reactive task, but for me, is part of the plan,
    % since the robot does not know its initial position, so it should
    % start wandering until it finds the beacon.
    function robotWandering
        
        % We are now using the PID controller for wandering too.
        % This makes wandering more time consuming, but enables us to
        % localize faster!
        if ~wandering_init
            turnAngle(serPort, .2, PID);
        else
            if LidarMmid <= 0.6 || approx(LidarL, 0.5)
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
        localization
    end

    % Planned Task - Following the path
    function robotFollowing

        % Obey your PID controller make a turn that equals P + I + D
        command = PID;
        
        % Detect if we make a sharp turn left
        if command >= 4
           heavy_turning = 1;
        else
           heavy_turning = 0;
        end
        
        turnAngle(serPort, .2, command);

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        localization
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
                lost_counter = 0;
            end
        elseif plan(1) == 'f'
            % Are we there yet?!
            % Instead of using odometry, I am using localization now!
            if (strcmp(currPos, 'BE') && SonReF(1) > 2.1 && SonReF(1) < 100)
               plan = [];
            end

            % Are we lost?! (checking if left side is >= 1m for 5 
            % consecutive measurements)
            if (LidarL >= 1.0)
               lost_counter = lost_counter + 1;
               if(lost_counter > 5)
                   plan = ['w', 'h', 'f'];
                   currPos = 'UNKNOWN';
                   currOrient = 'UNKNOWN';
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



    % The PID controller implementation, now in its own function!
    function O = PID
        % Now do a simple proportional action. This part is used as was
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
        if plan(1) ~= 'h'
            % Use the sonars for localization
            SonRead = ReadSonar(serPort, 1);
            if ~any(SonRead) 
                SonarR(1) = 100;
            else
                SonarR(1) = SonRead;
            end

            SonRead = ReadSonar(serPort, 2);
            if ~any(SonRead) 
                SonarF(1) = 100;
            else
                SonarF(1) = SonRead;
            end

            SonRead = ReadSonar(serPort, 3);
            if ~any(SonRead)
                SonarL(1) = 100;
            else
                SonarL(1) = SonRead;
            end

            % Check if the robot is currently in any of the red areas except E
            in_red_region = 0;
            if strcmp(currPos,'RB') || strcmp(currPos,'RC') || strcmp(currPos,'RD') || strcmp(currPos,'RA') || strcmp(currPos,'BC') || strcmp(currPos,'BD')
               in_red_region = 1; 
            end

            estimatedPos = {'UNKNOWN'};
            estimatedOrient = {'UNKNOWN'};
            blue_chaos = 0;
            if ~(heavy_turning && in_red_region)
                if approx(SonarL, 0.3) && SonarF >= 0.7
                    % Check if we are in a Blue region
                    if SonarR > 3
                       estimatedPos = {'BA' 'BB' 'BC' 'BD' 'BE' 'BF'};
                       estimatedOrient = {'U' 'D' 'L' 'L' 'L' 'U'};
                       blue_chaos = 1;
                    % Check if we are in Red E
                    elseif approx(SonarR, 2.3)
                        estimatedPos = {'RE'};
                        if SonarF <= 3 && SonarR > 1.2
                            estimatedOrient = {'D'};
                        else
                            estimatedOrient = {'U'}; 
                        end
                    % Check if we are in one of the other Red regions
                    elseif approx(SonarR, 1.3)
                        if SonarF > 3
                           estimatedPos = {'RA' 'RB' 'RC' 'RD'}; 
                           estimatedOrient = {'UNKNOWN' 'UNKNOWN' 'U' 'U'};
                        else
                           estimatedPos = {'RC' 'RD'}; 
                           estimatedOrient = {'D' 'D'};
                        end
                    end
                end
            else
               % Come here if we are in a red region and we are turning sharply
               if strcmp(currPos,'RB')
                   estimatedPos = {'GB'};
                   estimatedOrient = {'L'};
               elseif strcmp(currPos,'RC') || strcmp(currPos,'BC')
                   estimatedPos = {'GC'};
                   estimatedOrient = {'L'};
               elseif strcmp(currPos,'RD') || strcmp(currPos,'BD')
                   estimatedPos = {'GD'};
                   estimatedOrient = {'L'};
               elseif strcmp(currPos,'RA')
                   estimatedPos = {'GA'};
                   estimatedOrient = {'U'};
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
                    if strcmp(estimatedPos(1),last_pos_estimation) && ~strcmp(estimatedPos(i),'UNKNOWN')
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
                    currPos = estimatedPos(1);
                    currOrient = estimatedOrient(1);
                    last_pos_estimation = estimatedPos(1);
                    last_orient_estimation = estimatedOrient(1);
                elseif blue_chaos
                    if ~strcmp(last_written_pos, 'I know that I am in a blue region, but I am not sure in which one!\n')
                        fprintf(file,strcat('Current Task: ', plan(1)));
                        fprintf(file,'\n');
                        fprintf(file,'I know that I am in a blue region, but I am not sure in which one!\n');
                        fprintf(file,'----------------------------\n');
                        last_written_pos = 'I know that I am in a blue region, but I am not sure in which one!\n';
                        last_written_orient = 'NONE';
                    end
                end
            end
        else
            currPos = 'RA';
            currOrient = 'UNKNOWN';
        end
        if ~(strcmp(last_written_pos, currPos) && strcmp(last_written_orient, currOrient))
            fprintf(file,strcat('Current Task: ', plan(1)));
            fprintf(file,'\n');
            fprintf(file,strcat('Current Position: ', char(currPos)));
            fprintf(file,'\n');
            fprintf(file,strcat('Current Orientation: ', char(currOrient)));
            fprintf(file,'\n');
            fprintf(file,'----------------------------\n');
            last_written_pos = currPos;
            last_written_orient = currOrient;
        end
    end

    % A helper function to approximate values
    function result = approx(number1, number2)
        result = 0;
        if abs(number1 - number2) <= 0.15
            result = 1;
        end
    end
end
    