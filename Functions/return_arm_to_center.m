function return_arm_to_center
%% Initialisation Stuff
load('ArmVariables.mat');
initMotors;
terminateMotors;
initMotors;
%% Main Code
% Raise End Effector
move_single_motor(3,250);
pause(3);

% Straighten Arm
move_single_motor(1,180);
move_single_motor(2,180);
end