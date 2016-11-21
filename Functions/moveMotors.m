function moveMotors(id, goal_pos)
%Moves the motors in list ID to their goal positions (in degrees of their local axis)
%% Ininitialisation of Variables
axratio = 3.41;
mxratio = 11.375;
%% Find Current & Goal Angle for Each Motor then find the angular difference for each motor.
for x = 1:4
    if (id(x)==4)
        past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', id(x), 30); %Get Position Value from Dynamixel
        past_angle4 = past_dyna_degrees4/mxratio; %Convert dynamixel position to real world degrees
        degree_diff4 = abs(past_angle4-goal_pos(x)); %Find angular difference in the proposed movement
    elseif (id(x)==2)
        past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', id(x), 30);
        past_angle2 = (past_dyna_degrees2/axratio)+30;
        degree_diff2 = abs(past_angle2-goal_pos(x));
    elseif (id(x)==3)
        past_dyna_degrees3 = calllib('dynamixel','dxl_read_word', id(x), 30);
        past_angle3 = (past_dyna_degrees3/axratio)+30;
        degree_diff3 = abs(past_angle3-goal_pos(x));
    else
        past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', id(x), 30);
        past_angle1 = (past_dyna_degrees1/axratio)+30;
        degree_diff1 = abs(past_angle1-goal_pos(x));
    end
end
%Store Angular Difference for Each Motor & Find Max Change of all the
%Motors
degree_diff = [degree_diff1 degree_diff2 degree_diff3 degree_diff4];
max_degree_diff = max(degree_diff);
%Given largest angular change, find the 'time' for the motors to complete their movement
time = (max_degree_diff/180)*15;
%% Determine The Power for each motor such that they all take the same amount of time to move, then move motor
for x = 1:4
    if (id(x)==4)
        dyna_degrees = (goal_pos(x))*mxratio;
        power = (degree_diff(x)/1000)*1024/time;
        calllib('dynamixel', 'dxl_write_word', id(x), 32, power);
        calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
    else
        dyna_degrees = (goal_pos(x)-30)*axratio;
        power = (degree_diff(x)/360)*1024/time;
        if power<10;
            power = 10;
        end
        calllib('dynamixel', 'dxl_write_word', id(x), 32, power);
        calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
    end
end
%%Wait for all motors to finish moving
while (1)
    moving_1 = calllib('dynamixel','dxl_read_word', 1, 46);
    moving_2 = calllib('dynamixel','dxl_read_word', 2, 46);
    moving_3 = calllib('dynamixel','dxl_read_word', 3, 46);
    moving_4 = calllib('dynamixel','dxl_read_word', 4, 46);
    moving = [moving_1, moving_2, moving_3, moving_4];
    if max(moving) == 0
        break
    end
end
%% Motor Position Error Correction (get goal_pos actually = present_pos)
%Define the Goal Angle for each motor (only used motors 1 & 2 because the
%others were fine)
goal_pos_1 = calllib('dynamixel','dxl_read_word', 1, 30);
goal_pos_2 = calllib('dynamixel','dxl_read_word', 2, 30);
while (1)
    present_pos_1 = calllib('dynamixel','dxl_read_word', 1, 36); %Get current angle of both motors
    present_pos_2 = calllib('dynamixel','dxl_read_word', 2, 36);
    pos_error_1 = goal_pos_1-present_pos_1; %Find the error between the goal angle and current angle
    pos_error_2 = goal_pos_2-present_pos_2;
    error = [pos_error_1, pos_error_2]; %Combine Error into Vector
    if max(error)<=2 && min(error)>=-2 %Check if both errors are within +/- 2
        break %If so, break out of error loop as goal angle approx = current angle
    else %If error not within acceptable range, use the error and adjust the goal angle of each motor to overall reduce error
        if abs(error(1))>2
            calllib('dynamixel', 'dxl_write_word', 1, 32, 20);
            current_goal_pos = calllib('dynamixel','dxl_read_word', 1, 30);
            calllib('dynamixel','dxl_write_word', 1, 30, current_goal_pos+(error(1)*0.5));
        end
        if abs(error(2))>2
            calllib('dynamixel', 'dxl_write_word', 2, 32, 20);
            current_goal_pos = calllib('dynamixel','dxl_read_word', 2, 30);
            calllib('dynamixel','dxl_write_word', 2, 30, current_goal_pos+(error(2)*0.5));
        end
    end
%Wait for motors to finish moving after the new compensated angles have
%been driven to each motor.
while (1)
    moving_1 = calllib('dynamixel','dxl_read_word', 1, 46);
    moving_2 = calllib('dynamixel','dxl_read_word', 2, 46);
    moving = [moving_1, moving_2];
    if max(moving) == 0
        break %Once neither motor is moving, break out of this while and reloop through error to check if it is fixed
    end
end
end
end