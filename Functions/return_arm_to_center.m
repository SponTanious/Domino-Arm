function return_arm_to_center
%Straightens all the motors so the arm can be lined up with the workspace
%(points down the 'y' axis).
%% Initialisation Stuff
load('ArmVariables.mat');
initMotors;
%% Main Code
move_to_domino([0,(L2+L3+y0)*100,1.5,0]) %Max Reachble coordinate in Y axis such that the arm is straight.
end