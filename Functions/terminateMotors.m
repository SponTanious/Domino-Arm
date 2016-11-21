function terminateMotors()
calllib('dynamixel', 'dxl_write_word', 1, 24, 0);
calllib('dynamixel', 'dxl_write_word', 2, 24, 0);
calllib('dynamixel', 'dxl_write_word', 3, 24, 0);
calllib('dynamixel', 'dxl_write_word', 4, 24, 0);
calllib('dynamixel','dxl_terminate');
unloadlibrary('dynamixel');
end

