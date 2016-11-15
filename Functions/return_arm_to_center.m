function return_arm_to_center
% Raise End Effector
move_single_motor(3,250);
pause(3);

% Straighten Arm
move_single_motor(1,180);
move_single_motor(2,180);
end