function move_to_domino(points_lu)
load('ArmVariables.mat');

points_lu = transpose(points_lu);
initMotors;
terminateMotors;
initMotors;
pos = points_lu;

%% Move to above point
% Define Goal Pos:
x = pos(1)*0.01-x0;
y = pos(2)*0.01-y0;
z = (pos(3)+3)*0.01-z0;
phi3 = 180+z*1000*degree_per_mm;
move_single_motor(3,phi3);
pause(2);
move_single_motor(4,0);
pause(1);

%Calculate rotation in motor A, B & C:
phi22_1 = atan2( sqrt( 1-(((x)^2+(y)^2-(L2)^2-(L3)^2)/(2*L2*L3))^2 ), (x^2+y^2-(L2)^2-(L3)^2)/(2*(L2)*(L3)) );
phi22_2 = -atan2( sqrt( 1-(((x)^2+(y)^2-(L2)^2-(L3)^2)/(2*L2*L3))^2 ), (x^2+y^2-(L2)^2-(L3)^2)/(2*(L2)*(L3)) );
k1_1 = L2 + L3*cos(phi22_1);
k1_2 = L2 + L3*cos(phi22_2);
k2_1 = L3*sin(phi22_1);
k2_2 = L3*sin(phi22_2);
phi11_1 = atan2(y, x) - atan2(k2_1, k1_1);
phi11_2 = atan2(y, x) - atan2(k2_2, k1_2);

phi1_1 = ((phi11_1)*180/pi)+90;
phi2_1 = ((phi22_1)*180/pi)+180;
phi1_2 = ((phi11_2)*180/pi)+90;
phi2_2 = ((phi22_2)*180/pi)+180;

%% Check We Have The Right Elbow Solution, If not, change solution.
%Check which of the two solutions are valid
solution_1_elbow = 0;
solution_2_elbow = 0;
if 90<phi1_1 && phi1_1<270
    solution_1 = 1; %1 Means Valid solution
else
    solution_1 = 0;
end
if 90<phi1_2 && phi1_2<270
    solution_2 = 1;
else
    solution_2 = 0;
end
%Check there is at least 1 valid solution (if not, give error)
if solution_1 == 0 && solution_2 == 0
    error('Coordinate Not Physically Possible')
end
% Check Elbow of Valid Solutions
if solution_1 == 1
    if phi2_1>180 && phi1_1<180
        solution_1_elbow = 1; %1 Means Elbow Right   
    elseif phi2_1<180 && phi1_1<180
        solution_1_elbow = 2; %2 Means elbow Left   
    elseif phi1_1>180 && phi2_1<180
        solution_1_elbow = 2;   
    elseif phi1_1>180 && phi2_1>180
        solution_1_elbow = 1;
    end
end

if solution_2 == 1
    if phi2_2>180 && phi1_2<180
        solution_2_elbow = 1; %1 Means Elbow Right   
    elseif phi2_2<180 && phi1_2<180
        solution_2_elbow = 2; %2 Means elbow Left   
    elseif phi1_2>180 && phi2_2<180
        solution_2_elbow = 2;   
    elseif phi1_2>180 && phi2_2>180
        solution_2_elbow = 1;
    end
end
% Check if Previous Solution was elbow left or right
past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', 1, 30);
T1 = (past_dyna_degrees1/axratio)+30;
past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', 2, 30);
T2 = (past_dyna_degrees2/axratio)+30;

if T1<180 && T2>180
    past_solution_elbow = 1;
elseif T1<180 && T2<180
    past_solution_elbow = 2;
elseif T1>180 && T2>180
    past_solution_elbow = 1;
elseif T1>180 && T2<180
    past_solution_elbow = 2;
end

% Pick Between Solutions (if more than 1 is valid)
if past_solution_elbow == 1
    if solution_1_elbow == 1
        phi1 = phi1_1;
        phi2 = phi2_1;
    elseif solution_2_elbow == 1
        phi1 = phi1_2;
        phi2 = phi2_2;
    else
        if solution_1 == 1
            phi1 = phi1_1;
            phi2 = phi1_2;
        else
            phi1 = phi1_2;
            phi2 = phi2_2;
        end
    end
elseif past_solution_elbow == 2
    if solution_1_elbow == 2
        phi1 = phi1_1;
        phi2 = phi2_1;
    elseif solution_2_elbow == 2
        phi1 = phi1_2;
        phi2 = phi2_2;
    else
        if solution_1 == 1
            phi1 = phi1_1;
            phi2 = phi1_2;
        else
            phi1 = phi1_2;
            phi2 = phi2_2;
        end
    end
end
    
%% Finalize Angles & Move Motors
past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', 4, 30);
T4 = (past_dyna_degrees4/mxratio);
phi4 = T4;

moveMotors([1,2,3,4],[phi1,phi2,phi3,phi4]);
%% Drop Onto Point
z = (pos(3)+1)*0.01-z0;
phi3 = 180+z*1000*degree_per_mm;
move_single_motor(3,phi3);
pause(3)

phi4 = T4+180;
move_single_motor(4,phi4);
end