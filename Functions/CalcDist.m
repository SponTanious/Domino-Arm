function Dist = CalcDist(initial_L, goal_L)
% CalcDist calculates the distant between pixel locations, initial_L and goal_L
%   initial_L is 1x2 vectot
%   goal_L is 1x2 vector
    x_g = goal_L(1);
    y_g = goal_L(2);
    
    x_i = initial_L(1);
    y_i = initial_L(2);

    Dist = sqrt((x_i-x_g)^2 + (y_i-y_g)^2);

end