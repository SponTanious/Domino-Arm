function return_arm_to_center
%% Initialisation Stuff
load('ArmVariables.mat');
initMotors;
terminateMotors;
initMotors;
%% Main Code
move_to_domino([0,(L2+L3+y0)*100,1.5,0])
% % Raise End Effector
% move_single_motor(3,180);
% % Straighten Arm
% move_single_motor(1,180);
% move_single_motor(2,180);
end