function initMotors(numMotor)
loadlibrary('dynamixel', 'dynamixel.h');
libfunctions('dynamixel')
DEFAULT_PORTNUM = 3; %%com port
DEFAULT_BAUDNUM = 1; %% 1Mbps
calllib('dynamixel', 'dxl_initialize', DEFAULT_PORTNUM, DEFAULT_BAUDNUM);

for x = 1:numMotor
    calllib('dynamixel', 'dxl_write_word', x, 32, 50);
end

for x = 2:numMotor
    calllib('dynamixel', 'dxl_write_word', x, 6, 213);
    calllib('dynamixel', 'dxl_write_word', x, 8, 810);
end

end

