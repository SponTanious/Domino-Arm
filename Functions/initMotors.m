function initMotors
loadlibrary('dynamixel', 'dynamixel.h');
% libfunctions('dynamixel');
DEFAULT_PORTNUM = 3; %%com port
DEFAULT_BAUDNUM = 1; %% 1Mbps
calllib('dynamixel', 'dxl_initialize', DEFAULT_PORTNUM, DEFAULT_BAUDNUM);

for x = 1:4
    calllib('dynamixel', 'dxl_write_word', x, 32, 50); %%Set Power to 50
end
%% Set Angular Limit for Each Motor
% calllib('dynamixel', 'dxl_write_word', 4, 6, 0);
% calllib('dynamixel', 'dxl_write_word', 4, 8, 4095);

calllib('dynamixel', 'dxl_write_word', 1, 6, 170);
calllib('dynamixel', 'dxl_write_word', 1, 8, 900);

calllib('dynamixel', 'dxl_write_word', 2, 6, 15);
calllib('dynamixel', 'dxl_write_word', 2, 8, 1018);

calllib('dynamixel', 'dxl_write_word', 3, 6, 180);
calllib('dynamixel', 'dxl_write_word', 3, 8, 1023);

calllib('dynamixel', 'dxl_write_word', 1, 28, 128); % Set Compliance stuff for motor 1 & 2
calllib('dynamixel', 'dxl_write_word', 2, 28, 128);

calllib('dynamixel', 'dxl_write_word', 1, 29, 128);
calllib('dynamixel', 'dxl_write_word', 2, 29, 128);


calllib('dynamixel', 'dxl_write_word', 2, 14, 600); %%Set Torque Limit on Motor 2
end

