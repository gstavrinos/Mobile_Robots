function final_project(serPort)

    disp ('=============================')
    disp ('Final Project Robot Initiated')
    disp ('-----------------------------')

    % Sets forward velocity using differential system
    SetDriveWheelsCreate(serPort, 0.5, 0.5);

    % Proportional Gain
    Kp = 200;
    % Integral Gain
    Ki = 0.001;
    % Differential Gain
    Kd = 150;

    % Read all 4 sonar values
    SonRead = ReadSonar(serPort, 1);
    if ~any(SonRead) 
        SonRiF(1) = 100;
    else
        SonRiF(1) = SonRead;
    end

    SonRead = ReadSonar(serPort, 2);
    if ~any(SonRead) 
        SonFF(1) = 100;
    else
        SonFF(1) = SonRead;
    end

    SonRead = ReadSonar(serPort, 3);
    if ~any(SonRead)
        SonLF(1) = 100;
    else
        SonLF(1) = SonRead;
    end

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
    % Read Left side of Lidar
    [LidarL, LidarD] = min(LidarRes(341:681));

    % Read Right side of Lidar
    [LidarR, LidarD2] = min(LidarRes(1:341));


    % Check front ~45 degrees
    [LidarMmid, LidarDmid] = min(LidarRes(216:466));

    % The possible tasks are coded like this:
    % 0 = find the center of the starting room
    % 1 = find the exit of the starting room
    % 2 = exit the starting room
    % 3 = find beacon
    % 4 = approach beacon
    % 5 = find the entrance to the starting room
    % 6 = enter the starting room
    
    % This our plan. The sequence of tasks.
    plan = [0, 1, 2, 3, 4, 5, 6];
    
    % Obstacle Avoidance is disabled for now
    %obstacle_avoidance = 0;

    % Initialize the Integral buffer
    e100 = zeros(100,1);

    % Iteration initialized to zero. It is used as a timer.
    t = 0;
    
    % Lost counter initialized to zero. It is used to get back to wandering
    % mode.
    lost_counter = 0;
    
    % Variable used to check if the robot is exiting or entering the
    % starting room
    exiting = 1;

    % Main loop
    while(size(plan) > 0)

        pause(.1)

        % Reset obstacle avoidance
        %obstacle_avoidance = 0;
        
        % Increase timer
        t = t + 1;

        LidarRes = LidarSensorCreate(serPort);
        
        % Read odometry sensor
        % Give some extra time for the odom reading. Without this extra
        % pause, the odometry reading does not seem to return anything!
        pause(.1)

        % Read Left side of Lidar
        [LidarL, LidarD] = min(LidarRes(341:681));

        % Read Right side of Lidar
        [LidarR, LidarD2] = min(LidarRes(1:341));
        
        
        % Check front ~45 degrees
        [LidarMmid, LidarDmid] = min(LidarRes(216:466));
        
        Camera = CameraSensorCreate(serPort);

        SonRead = ReadSonar(serPort, 1);
        if ~any(SonRead) 
            SonRiF(1) = 100;
        else
            SonRiF(1) = SonRead;
        end

        SonRead = ReadSonar(serPort, 2);
        if ~any(SonRead) 
            SonFF(1) = 100;
        else
            SonFF(1) = SonRead;
        end

        SonRead = ReadSonar(serPort, 3);
        if ~any(SonRead)
            SonLF(1) = 100;
        else
            SonLF(1) = SonRead;
        end

        SonRead = ReadSonar(serPort, 4);
        if ~any(SonRead) 
            SonReF(1) = 100;
        else
            SonReF(1) = SonRead;
        end
        
        % Don't change anything on the planned tasks if we had to run a
        % reactive task in this loop
        %if ~obstacle_avoidance
            % Change the plan based on our new environment
            %planManager
            
            % Run the corresponding planned task
            if size(plan) > 0
                % Task-1: Wandering mode
                if plan(1) == 0
                    findCenter
                % Task-2: Homing mode
                %elseif plan(1) == 'h'
                %    robotHoming
                % Task-3: Following mode
                %elseif plan(1) == 'f'
                %    robotFollowing
                end
            end
        %end
    end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('I reached my goal! :)');

    
    % ------------------------------------------------------------------
    
    
    % Helper functions
    function findCenter
        
        % Override the PID controller, to make the process faster, and
        % safer!
        if SonFF < 0.3
            turnAngle(serPort, .2, 90);
        end
        
        % Now do a simple porportional action. This part is used as was
        % provided, but with a change on the variable names.
        et = LidarL - LidarR;
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

        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
        
        % This should go inside the plan manager, but let's test it here
        % for now!
        if exiting
            if SonRiF == 100 || SonLF == 100 % The robot is facing down or up!
                if approx(SonReF,SonFF)
                   % I should change the plan here in the future!
                   disp('We are in the center of the room!') 
                end
            elseif SonFF == 100 || SonReF == 100 % The robot is facing left or right!
                if approx(SonRiF,SonLF)
                   % I should change the plan here in the future!
                   disp('We are in the center of the room!')
                end
            end
        end
        
    end

    % This is a method that manages the different planned tasks. This could
    % be described as a reactive task, although it is related to the
    % requirements of the plan.
%     function planManager
%         if plan(1) == 'w'
%             % Check the camera for the beacon
%             if(any(Camera))
%                plan = ['h', 'f'];
%                disp('Saw beacon! Starting Task 2 - homing');
%             end
%         elseif plan(1) == 'h'
%             if(~any(Camera))
%                 plan = 'f';
%                 disp('Passed the beacon. Starting Task 3 - following');
%                 Dist1 = 0;
%                 lost_counter = 0;
%             end
%         elseif plan(1) == 'f'
%             % Are we there yet?!
%             if (Dist1 >= 17 && SonReF(1) > 2.1 && SonReF(1) < 100)
%                plan = [];
%             end
% 
%             % Are we lost?! (checking if left side is >= 1m for 5 
%             % consecutive measurements)
%             if (LidarM >= 1.0)
%                lost_counter = lost_counter + 1;
%                if(lost_counter > 5)
%                    plan = ['w', 'h', 'f'];
%                    disp('I am lost! Starting Task 1 - wandering');
%                end
%             else
%                 % Reset lost_counter. We need 5 consecutive "lost" conditions.
%                 lost_counter = 0;
%             end
%         end
%     end

% A helper function to approximate values
    function result = approx(number1, number2)
        result = 0;
        if abs(number1 - number2) <= 0.1
            result = 1;
        end
end

end
    