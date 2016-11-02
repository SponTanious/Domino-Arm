function moveMotors_Simultaneous (id, goal_pos)
%MOVEMOTORS moves the motors in list ID to their GOAL_POS
axratio = 3.41;
mxratio = 11.377;
a = size(id);

% Make syncwrite packet
calllib('dynamixel','dxl_set_txpacket_id',254);
calllib('dynamixel','dxl_set_txpacket_instruction',131);
calllib('dynamixel','dxl_set_txpacket_parameter',0,30);
calllib('dynamixel','dxl_set_txpacket_parameter',1,2);
for i = 0:1:4-1
 calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i,id(i+1));
    if (id(i+1) == 1)
        dyna_degrees = (goal_pos(i+1))*mxratio;

    else
        dyna_degrees = (goal_pos(i+1)-30)*axratio;
       
    end
               Position(i+1) = int32(dyna_degrees);
               low = calllib('dynamixel','dxl_get_lowbyte',dyna_degrees);
               calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i+1,low);
               high = calllib('dynamixel','dxl_get_highbyte',dyna_degrees);
               calllib('dynamixel','dxl_set_txpacket_parameter',2+3*i+2,high);
end
calllib('dynamixel','dxl_set_txpacket_length',(2+1)*4+4);
calllib('dynamixel','dxl_txrx_packet');


% for x = 1:a(2)
%     if (id(x) == 1)
%         dyna_degrees = (goal_pos(x))*mxratio;
%         calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
%    
%     else
%         dyna_degrees = (goal_pos(x)-30)*axratio;
%         calllib('dynamixel','dxl_write_word', id(x), 30, dyna_degrees);
%     end
% end
end