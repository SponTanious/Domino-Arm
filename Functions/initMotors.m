function initMotors
%Initialises the connection between Matlab & Dynamixel Motors, loads the
%dynamixel library of function (used in driving the motors) and sets
%angular limits for each motor.
loadlibrary('dynamixel', 'dynamixel.h');
DEFAULT_PORTNUM = 3; %%USB com port of motor cable
DEFAULT_BAUDNUM = 1; %% 1Mbps
calllib('dynamixel', 'dxl_initialize', DEFAULT_PORTNUM, DEFAULT_BAUDNUM);

for x = 1:4
    calllib('dynamixel', 'dxl_write_word', x, 32, 50); %%Set Power to 50 for each motor (not too fast)
end
%% Set Angular Limit for Each Motor (based on arm geometry and max position in workspace)
calllib('dynamixel', 'dxl_write_word', 1, 6, 170);
calllib('dynamixel', 'dxl_write_word', 1, 8, 900);

calllib('dynamixel', 'dxl_write_word', 2, 6, 15);
calllib('dynamixel', 'dxl_write_word', 2, 8, 1018);

calllib('dynamixel', 'dxl_write_word', 3, 6, 180);
calllib('dynamixel', 'dxl_write_word', 3, 8, 1023);
%No angle limit for motor 4 as it can/will do full 360 revolution.

%Also need to manually set the compliance slope for motor 1 & 2 the first
%time the motors are initialised (using Dynamixel Wizard) (can't be done through MATLAB)
end