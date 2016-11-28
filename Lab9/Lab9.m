function Lab9(serPort)

    disp ('====================')
    disp ('Lab9 Robot Initiated')
    disp ('--------------------')
    
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
    [LidarM, LidarD] = min(LidarRes(341:681));
    
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
        [LidarM, LidarD] = min(LidarRes(341:681));

        % Read Right side of Lidar
        [LidarM2, LidarD2] = min(LidarRes(1:341));
        
        
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
        % If there is not enough space in front of the robot
        if (LidarRes(341) < 1.0)
            if ( LidarM < LidarM2) 
                turnAngle(serPort, .2, -40);
            else
                turnAngle(serPort, .2, 40);
            end
        end
        % Check the sides using the LIDAR and try to stay away from walls
        if ( LidarM < LidarM2 && LidarM < 0.5) 
            turnAngle(serPort, .2, -10);
        elseif (LidarM2 > LidarM2 && LidarM2 < 0.5)
            turnAngle(serPort, .2, 10);
        end

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
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
        % Now do a simple porportional action. This part is used as was
        % provided, but with a change on the variable names.
        et = LidarM - 0.5;
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

        % Obey your PID controller make a turn that equals P + I + D
        turnAngle(serPort, .2, O);

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
            if (LidarM >= 1.0)
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
end
    