function initMotors
loadlibrary('dynamixel', 'dynamixel.h');
% libfunctions('dynamixel');
DEFAULT_PORTNUM = 4; %%com port
DEFAULT_BAUDNUM = 1; %% 1Mbps
calllib('dynamixel', 'dxl_initialize', DEFAULT_PORTNUM, DEFAULT_BAUDNUM);

for x = 1:4
    calllib('dynamixel', 'dxl_write_word', x, 32, 50); %%Set Power to 50
end
%% Set Angular Limit for Each Motor
calllib('dynamixel', 'dxl_write_word', 4, 6, 0);
calllib('dynamixel', 'dxl_write_word', 4, 8, 4095);

calllib('dynamixel', 'dxl_write_word', 1, 6, 200);
calllib('dynamixel', 'dxl_write_word', 1, 8, 830);

calllib('dynamixel', 'dxl_write_word', 2, 6, 15);
calllib('dynamixel', 'dxl_write_word', 2, 8, 1018);

calllib('dynamixel', 'dxl_write_word', 3, 6, 180);
calllib('dynamixel', 'dxl_write_word', 3, 8, 1023);
end

