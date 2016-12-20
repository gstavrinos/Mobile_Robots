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
    [LidarLall, LidarD] = min(LidarRes(341:681));
    [LidarL, LidarD] = min(LidarRes(441:681));  % use only the sides

    % Read Right side of Lidar
    [LidarRall, LidarD2] = min(LidarRes(1:341));
    [LidarR, LidarD2] = min(LidarRes(1:241)); % use only the sides



    % Check front ~45 degrees
    [LidarMmid, LidarDmid] = min(LidarRes(216:466));

    % The possible tasks are coded like this:
    % 0 = find the center of the starting room
    % 1 = face the exit of the starting room
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
    % starting room.
    exiting = 1;
    
    % Variable used to check if the beacon is near the door of the starting
    % room.
    check_surroundings = 1;
    
    % A counter to estimate (roughly) the angles we have turned during the
    % check_surroundings procedure.
    angle_cnt = 0;

    % Main loop
    while(size(plan) > 0)

        pause(.1)

        % Reset obstacle avoidance
        %obstacle_avoidance = 0;
        
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
            planManager
            
            % Run the corresponding planned task
            if size(plan) > 0
                % Task-1: Centering in starting room
                if plan(1) == 0 || plan(1) == 2
        
                    % Override the PID controller, to make the process faster, and
                    % safer!
                    if SonFF < 0.5
                        turnAngle(serPort, .2, 70);
                        t = 1;
                    end
                    PID_controller(LidarL/4, LidarR/4);
                elseif plan(1) == 1
                    door_alignment
                elseif plan(1) == 3
                    if check_surroundings
                        if angle_cnt < 300
                           turnAngle(serPort, .2, 10);
                           angle_cnt = angle_cnt + 10;
                        elseif approx(LidarMmid,2,.2)
                            check_surroundings = 0;
                        else
                            SetDriveWheelsCreate(serPort, .5, .5);
                        end
                    else
                       PID_controller(LidarL/4, LidarR/4);
                    end
                end
            end
        %end
    end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('I reached my goal! :)');

    
    % ------------------------------------------------------------------
    
    
    % Helper functions
    function PID_controller(value1, value2)
        % Now do a simple porportional action. This part is used as was
        % provided, but with a change on the variable names.
        et = value1 - value2;
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
    end

    function door_alignment
       if exiting
           if SonFF < 100
              turnAngle(serPort, .2, 10); 
           end
       else %entering
           
       end
    end

    % This is a method that manages the different planned tasks. This could
    % be described as a reactive task, although it is related to the
    % requirements of the plan.
    function planManager
        if plan(1) == 0
            if exiting
                if SonRiF == 100 || SonLF == 100 % The robot is facing down or up!
                    if approx(SonReF,SonFF,.1) && (approx(SonRiF,1.5,.2) || approx(SonLF,1.5,.2))
                       plan = [1, 2, 3, 4, 5, 6];
                        disp('CENTEEEEER');
                    end
                elseif SonFF == 100 || SonReF == 100 % The robot is facing left or right!
                    if approx(SonRiF,SonLF,.1) && (approx(SonFF,2,.2) || approx(SonFF,2,.2))
                       plan = [1, 2, 3, 4, 5, 6];
                        disp('CENTEEEEER');
                    end
                elseif approx(SonFF,SonReF,.1) && approx(SonRiF,SonLF,.1)
                    plan = [1, 2, 3, 4, 5, 6];
                    disp('CENTEEEEER');
                end
            end
            
            % Check the camera for the beacon
%             if(any(Camera))
%                plan = ['h', 'f'];
%                disp('Saw beacon! Starting Task 2 - homing');
%             end
        elseif plan(1) == 1 && SonFF == 100
            disp('Aligned with door!');
            plan = [2, 3, 4, 5, 6];
        elseif plan(1) == 2 && approx(SonReF,3,.1)
            disp('I am out of the door!');
            exiting = 0;
            plan = [3, 4, 5, 6];
        elseif plan(1) == 3
            if(any(Camera))
               plan = [4, 5, 6];
               disp('I found the beacon!');
            end

            % Are we lost?! (checking if left side is >= 1m for 5 
            % consecutive measurements)
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
        end
    end

% A helper function to approximate values
    function result = approx(number1, number2, limit)
        result = 0;
        if abs(number1 - number2) <= limit
            result = 1;
        end
end

end
    