function move_arm_out_of_view
move_single_motor(3,300); %Raise End Effector then wait
pause(3);
% Straighten Arms
move_single_motor(1,90);
move_single_motor(2,205);
end