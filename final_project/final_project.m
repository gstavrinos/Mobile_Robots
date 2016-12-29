function final_project(serPort)

    disp ('===============================')
    disp ('|Final Project Robot Initiated|')
    disp ('-------------------------------')

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
    
    % Read bumper and drop sensors
    [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    
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

    % Get camera reading
    [Camera, bDist] = CameraSensorCreate(serPort);

    % Check front ~45 degrees
    [LidarMmid, LidarDmid] = min(LidarRes(216:466));

    % The possible tasks are coded like this:
    % 0 = find the center of the starting room
    % 1 = face the door of the starting room
    % 2 = exit the starting room
    % 3 = find beacon
    % 4.1 = approach the beacon
    % 4.2 = perpendicularly bump the beacon
    % 5 = find the entrance to the starting room
    % 6 = reach the goal
    
    % This our plan. The sequence of planned tasks.
    plan = [ 4.1, 4.2, 5, 1, 6];
    
    % Obstacle Avoidance is disabled for now
    obstacle_avoidance = 0;

    % Initialize the Integral buffer
    e100 = zeros(100,1);

    % Iteration initialized to zero. It is used as a timer.
    t = 0;
    
    % Variable used to check if the robot is exiting or entering the
    % starting room.
    exiting = 0;%TODO change to 1 after tests
    
    % Variable used to check if the beacon is near the door of the starting
    % room.
    check_surroundings = 0;%TODO change to 1 after tests
    
    % A counter to estimate (roughly) the angles we have turned during the
    % check_surroundings procedure.
    angle_cnt = 0;
    
    % Variable used to know if wandering is used to return to the starting
    % room
    get_back = 0;
    
    % Variable to check if the robot started facing the starting room
    facing_wall = 0;
    
    % Variable that states the side of the found door
    door_on_the_left = 1;
    
    % Variable that states that our last 'blind' turn was a left one
    left_turn = 0;
    
    % Lost beacon counter that is used to make sure that we have really
    % lost sight of the beacon
    b_cnt = 0;
    
    % Variable that states if the robot is aligned with the beacon's wall
    aligned = 0;

    % Main loop
    while(size(plan) > 0)

        pause(.1)

        updateSensors
        
        % Reset obstacle avoidance
        obstacle_avoidance = 0;
        
        % Increase timer
        t = t + 1;
        
        % Change the plan based on our new environment
        planManager

        if size(plan) > 0
            % Run the corresponding planned task
            % Task-0: Center in starting room
            % Task-2: Exit the starting room
            if plan(1) == 0 || plan(1) == 2
                center
            % Task-1: Align with the door
            elseif plan(1) == 1
                door_alignment
            % Task-3: Search for the beacon
            % Task-5: Search for the entrance
            elseif plan(1) == 3 || plan(1) == 5
                if ~get_back % Do not try to avoid the bumped 'obstacle!'
                    obstacleAvoidance
                end
                if ~obstacle_avoidance
                    wander
                end
            % Task-4.1: Approach the beacon
            elseif plan(1) == 4.1
                obstacleAvoidance
                if ~obstacle_avoidance
                    approachBeacon
                    bDist = bDist
                end
            % Task-4.2: Bump the beacon
            elseif plan(1) == 4.2
                SetDriveWheelsCreate(serPort, 0, 0);
                bumpBeacon
            % Task-6: Reach the goal
            elseif plan(1) == 6
                reach_goal
            end
        end
    end

    % Stop motors
    SetDriveWheelsCreate(serPort, 0, 0);
    disp('I reached my goal! :)');

    
    % ------------------------------------------------------------------
    
    
    % Final Planned Task - Reach the goal
    function reach_goal
        if LidarRes(341) < 3.5
            if SonLF < 3 && SonLF > 1 && approx(SonLF, SonRiF,.1)
                SetDriveWheelsCreate(serPort, 0.5, 0.5);
            else
                center
            end
        else
            SetDriveWheelsCreate(serPort, 0.5, 0.5);
        end
    end

    function approachBeacon
        if (any(Camera) && abs(Camera) > 0.05)
            turnAngle (serPort, .2, (Camera * 6));
        end 
        % Reset straight line and advance
        SetDriveWheelsCreate(serPort, 0.5, 0.5);
    end

    % Planned Task - Bump the beacon!
    function bumpBeacon
        % The simple homing task (without obstacle avoidance) is used as
        % it was provided.
%         if (any(Camera) && abs(Camera) > 0.05)
%             turnAngle (serPort, .2, (Camera * 180/3.1415));
%         end 
%        if any(Camera)
%             if approx(LidarMmid, bDist, .5)
%                 beacon_laser_index = fix(341 - Camera * 180/3.1415 * (120/681))
%                 %PID_controller(LidarRes(beacon_laser_index-10),LidarRes(beacon_laser_index+10))
%                 %PID_controller(max(LidarRes(beacon_laser_index-10:beacon_laser_index)),max(LidarRes(beacon_laser_index:beacon_laser_index+10)))
%                 min_ = 100;
%                 max_ = -1;
%                 for i=beacon_laser_index-10:beacon_laser_index+10
%                    if 
%                 end
%                 min(LidarRes(beacon_laser_index-10:beacon_laser_index+10))
%                 max(LidarRes(beacon_laser_index-10:beacon_laser_index+10))
%                 bDist
%                 %PID_controller(min(LidarRes(beacon_laser_index-10:beacon_laser_index+10)),bDist)
%             else
%                 if abs(Camera) > 0.05
%                     turnAngle (serPort, .2, (Camera * 6));
%                 end
%             end
%             
%         end
        i1 = 0;
        i2 = 0;
        l_space = 0;
        r_space = 0;
        cam = 0;
        [c, beacon_laser_index] = min(LidarRes)
        %if any(Camera) && bDist < 4
            b_cnt = 0;
            cam = Camera * 180/pi
            %beacon_laser_index = round(341 + Camera * 180/pi * (681/240))
            %c = min(LidarRes(beacon_laser_index-1:beacon_laser_index+1))
            bDist = bDist
            prev_v = c;
            %LidarRes
            i1 = beacon_laser_index;
            i2 = beacon_laser_index;
            %limit = min (0.5, LidarRes(i)/10)
            limit = bDist/50;
            limit = 0.002;
            for i=beacon_laser_index:-1:1
                if abs(LidarRes(i)-prev_v) > limit || LidarRes(i) == 4
                   i1 = i+1;
                   break;
                end
                prev_v = LidarRes(i);
            end
            prev_v = c;
            for i=beacon_laser_index:681
                if abs(LidarRes(i)-prev_v) > limit || LidarRes(i) == 4
                   i2 = i-1;
                   break;
                end
                prev_v = LidarRes(i);
            end
            i1 = i1
            i2 = i2
            
            m = min(LidarRes(i1:i2));
            if i1 > 0 && i2 > 0
                l1 = LidarRes(i1)
                l2 = LidarRes(i2)
            end
            % if the cluster is not big enough (less than 100 laser pings)
            if abs(i1-i2) < 100
                disp('small cluster');
                % Check the side with the most space to move!
%                 l_space = sum(LidarRes(341:681));
%                 r_space = sum(LidarRes(1:341));
%                 if l_space > r_space %&& abs(cam) < 40
%                     disp('LEEEEFT');
%                    turnAngle(serPort, .2, 5)
%                    left_turn = 1;
%                 elseif r_space > l_space %&& abs(cam) < 40
%                     disp('RIIIIGHT');
%                    turnAngle(serPort, .2, -5) 
%                    left_turn = 0;
%                 end
                is_left = 1;
                if beacon_laser_index < 341
                    is_left = 0;
                end
                if is_left && c > 0.6
                    disp('LEEEEFT');
                    turnAngle(serPort, .2, 5)
                elseif ~is_left || c > 0.6
                    disp('RIIIIGHT');
                    turnAngle(serPort, .2, -5)
                end
            else
               disp('big cluster! all good!');
               sel_index = i1;
                if LidarRes(i2) > LidarRes(i1)
                   sel_index = i2;
                end
                if ~approx(l1, l2, 0.05) && ~aligned
                    ang = -(341-sel_index)*240/681
                    % If we are close enough, just align with the center of
                    % the wall!
                    if approx(l1, l2, 0.05) && LidarRes(341) < 0.3
                        turnAngle(serPort, .2, ang/4);
                        disp('aligned!');
                        aligned = 1;
                    else
                       turnAngle(serPort, .2, ang/2); 
                    end
                end
            end
        %end
        %if abs(cam) < 40
            SetDriveWheelsCreate(serPort, 0.1, 0.1);
        %else
        %   turnAngle(serPort, .2, cam/2); 
        %end
    end
    
    % Planned Task - Explore the map
    function wander
        if get_back
            SetDriveWheelsCreate(serPort, 0, 0);
            pause(.1)
            SetDriveWheelsCreate(serPort, -0.5, -0.5);
            pause(.5)
            turnAngle(serPort, .2, 180);
            get_back = 0;
        elseif check_surroundings
            if angle_cnt < 300
               turnAngle(serPort, .2, 10);
               angle_cnt = angle_cnt + 10;
            elseif approx(LidarMmid,2,.2)
                check_surroundings = 0;
            else
                SetDriveWheelsCreate(serPort, .5, .5);
            end
        else
           center
        end
    end

    % Planned Task - Align with the door
    function door_alignment
       if exiting
           if SonFF < 100
              turnAngle(serPort, .2, 10); 
           end
       else %entering
          if door_on_the_left
            turnAngle(serPort, .2, 10); 
          else
            turnAngle(serPort, .2, -10);
          end;
           if SonFF < 100 && ~facing_wall
               facing_wall = 1;
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
    
    % Helper function that calls the PID controller (part of the planned tasks)
    function center
        % Override the PID controller, to make the process faster, and
        % safer!
        if SonFF < 0.5
            turnAngle(serPort, .2, 70);
            t = 1;
        end
        PID_controller(LidarL/4, LidarR/4);
    end
    
    % The PID Controller implementation
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

    % This is a method that manages the different planned tasks. This could
    % be described as a reactive task, although it is related to the
    % requirements of the plan.
    function planManager
        %plan(1)
        if plan(1) == 0
            if exiting
                if SonRiF == 100 || SonLF == 100 % The robot is facing down or up!
                    if approx(SonReF,SonFF,.1) && (approx(SonRiF,1.5,.2) || approx(SonLF,1.5,.2))
                       plan = [1, 2, 3, 4.1, 4.2, 5, 1, 6];
                    end
                elseif SonFF == 100 || SonReF == 100 % The robot is facing left or right!
                    if approx(SonRiF,SonLF,.1) && approx(SonFF,2,.2)
                       plan = [1, 2, 3, 4.1, 4.2, 5, 1, 6];
                    end
                elseif approx(SonFF,SonReF,.1) && approx(SonRiF,SonLF,.1)
                    plan = [1, 2, 3, 4.1, 4.2, 5, 1, 6];
                end
            end
        elseif plan(1) == 1
            if exiting && SonFF == 100
                plan = [2, 3, 4.1, 4.2, 5, 1, 6];
            elseif facing_wall && SonFF == 100
                t = 1;
                plan = [6];
            end
        elseif plan(1) == 2 && approx(SonReF,3,.1)
            exiting = 0;
            plan = [3, 4.1, 4.2, 5, 1, 6];
        elseif plan(1) == 3
            if(any(Camera))
               plan = [4.1, 4.2, 5, 1, 6];
            end
        elseif plan(1) == 4.1 & bDist <= 0.8
            plan = [4.2, 5, 1, 6];
        elseif plan(1) == 4.2 && BumpFront
            plan = [5, 1, 6];
            get_back = 1;
        elseif plan(1) == 5
            if (SonRiF == 100 || SonLF == 100) && approx(LidarRes(1),LidarRes(681),0.1) && approx(LidarRes(170),LidarRes(511),0.1)
                if(SonRiF == 100)
                   door_on_the_left = 0; 
                else
                    door_on_the_left = 1;
                end
               plan = [1, 6];
            end
        elseif plan(1) == 6
            if approx(SonFF,1.5,.05) && SonLF < 100 && SonReF == 100
               plan = [];
            end
        end
    end

    % A helper function to approximate values
    function result = approx(number1, number2, limit)
        result = 0;
        if abs(number1 - number2) <= limit
            result = 1;
        end
    end

    % We update all sensors here
    function updateSensors
        LidarRes = LidarSensorCreate(serPort);

        % Read Left side of Lidar
        [LidarL, LidarD] = min(LidarRes(341:681));

        % Read Right side of Lidar
        [LidarR, LidarD2] = min(LidarRes(1:341));
        
        % Check front ~45 degrees
        [LidarMmid, LidarDmid] = min(LidarRes(216:466));
        
        [Camera, bDist] = CameraSensorCreate(serPort);

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
        
        [BumpRight, BumpLeft, WheDropRight, WheDropLeft, WheDropCaster, BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        
    end

end
    