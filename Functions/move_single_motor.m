function move_single_motor (id, goal_pos)
axratio = 3.41;
mxratio = 11.377;
 if (id==4)
        dyna_degrees = (goal_pos)*mxratio;
        calllib('dynamixel', 'dxl_write_word', id, 32, 50);
        calllib('dynamixel','dxl_write_word', id, 30, dyna_degrees);
 else
        dyna_degrees = (goal_pos-30)*axratio;
        calllib('dynamixel', 'dxl_write_word', id, 32, 50);
        calllib('dynamixel','dxl_write_word', id, 30, dyna_degrees);
 end
end