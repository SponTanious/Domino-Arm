function initMotors(numMotor)
loadlibrary('dynamixel', 'dynamixel.h');
libfunctions('dynamixel')
DEFAULT_PORTNUM = 4; %%com port
DEFAULT_BAUDNUM = 1; %% 1Mbps
calllib('dynamixel', 'dxl_initialize', DEFAULT_PORTNUM, DEFAULT_BAUDNUM);

for x = 1:numMotor
    calllib('dynamixel', 'dxl_write_word', x, 32, 50);
end

calllib('dynamixel', 'dxl_write_word', 2, 6, 213);
calllib('dynamixel', 'dxl_write_word', 2, 8, 512);

calllib('dynamixel', 'dxl_write_word', 3, 6, 240);
calllib('dynamixel', 'dxl_write_word', 3, 8, 810);

calllib('dynamixel', 'dxl_write_word', 4, 6, 190);
calllib('dynamixel', 'dxl_write_word', 4, 8, 500);


end

