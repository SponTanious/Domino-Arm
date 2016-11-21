function terminateMotors()
%Turns the Torque off on each motor and terminates the connection with
%Matlab. This allows us to manualy move the arm/motors for testing.
calllib('dynamixel', 'dxl_write_word', 1, 24, 0); %Holding Torque to 0
calllib('dynamixel', 'dxl_write_word', 2, 24, 0);%Holding Torque to 0
calllib('dynamixel', 'dxl_write_word', 3, 24, 0);%Holding Torque to 0
calllib('dynamixel', 'dxl_write_word', 4, 24, 0);%Holding Torque to 0
calllib('dynamixel','dxl_terminate'); %Terminate the communication
unloadlibrary('dynamixel'); %Unload the Dynamixel Librar
end

