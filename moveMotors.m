function moveMotors(id, goal_pos)
%MOVEMOTORS moves the motors in list ID to their GOAL_POS
axratio = 3.41;
mxratio = 11.377;
a = size(id);

for x = 1:a(2)
    if (id(x) == 1)
        dyna_degrees = (goal_pos(x))*mxratio
        calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
   
    else
        dyna_degrees = (goal_pos(x)-30)*axratio
        calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
    end
end

end