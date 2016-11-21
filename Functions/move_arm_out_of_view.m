function move_arm_out_of_view
%When using the camera to detect domunos/motion plan we can't have the arm
%in the way so this function quickly moves the arm off the to the side of
%the workspace (out of the view of the camera).
%% Main Code
initMotors;
move_single_motor(3,300); %Raise End Effector to above workspace
move_single_motor(1,90); %Moves arm 1
move_single_motor(2,205); %Moves arm 2
end