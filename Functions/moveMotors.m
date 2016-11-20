function moveMotors(id, goal_pos)
%MOVEMOTORS moves the motors in list ID to their GOAL_POS (anglular
%position)
axratio = 3.41;
mxratio = 11.375;
initMotors; %Can Delete Once Main Code is Finished
%% Find Current/Previous Angle of Each Motor then Angular difference to the Desired Angle
for x = 1:4
    if (id(x)==4)
        past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', id(x), 30);
        past_angle4 = past_dyna_degrees4/mxratio;
        degree_diff4 = abs(past_angle4-goal_pos(x));
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
%Given largest angular change, find the time to complete movement
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
%% Motor Position Error Correction (get goal_pos = present_pos)
goal_pos_1 = calllib('dynamixel','dxl_read_word', 1, 30)
goal_pos_2 = calllib('dynamixel','dxl_read_word', 2, 30)
while (1)
    present_pos_1 = calllib('dynamixel','dxl_read_word', 1, 36);
    present_pos_2 = calllib('dynamixel','dxl_read_word', 2, 36);
    pos_error_1 = goal_pos_1-present_pos_1;
    pos_error_2 = goal_pos_2-present_pos_2;
    error = [pos_error_1, pos_error_2]
    if max(error)<=2 && min(error)>=-2
        break
    else
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
    %Wait for all motors to finish moving
while (1)
    moving_1 = calllib('dynamixel','dxl_read_word', 1, 46);
    moving_2 = calllib('dynamixel','dxl_read_word', 2, 46);
    moving = [moving_1, moving_2];
    if max(moving) == 0
        break
    end
end
end
end