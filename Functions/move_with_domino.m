function move_with_domino(points_lu)
%Once the end effector is already resting on a domino (with some pressure
%applied), hold onto that domino and move between a path of vectors to a
%final destination.
%% Initialisation Stuff
load('ArmVariables.mat');
number_of_coords = size(points_lu);
number_of_coords = number_of_coords(1);
points_lu = transpose(points_lu);
count = 0;
initMotors;
%% Loop through a series of coordinates to move the arm to
for pos = points_lu
%For inout, define coordiantes in m.
x = pos(1)*0.01-x0;
y = pos(2)*0.01-y0;
z = (pos(3)-0.5)*0.01-z0;
%% Find All possible angles for each motor given the coordinate
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
%Check which of the two arm solutions are valid
solution_1_elbow = 0;
solution_2_elbow = 0;
if 90<phi1_1 && phi1_1<270
    solution_1 = 1; %1 Means Valid solution
else
    solution_1 = 0; %0 Means non-valid solution
end
if 90<phi1_2 && phi1_2<270
    solution_2 = 1;
else
    solution_2 = 0;
end
%Check there is at least 1 valid solution (if not, return error)
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
% Check if Previous Solution (arm position) was elbow left or right
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

% Pick Between Solutions (if more than 1 is valid) so it matches the past
% elbow solution (if possible).
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
%Get All the past/current anges of the motors
past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', 1, 30);
T1 = (past_dyna_degrees1/axratio)+30;
past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', 2, 30);
T2 = (past_dyna_degrees2/axratio)+30;
past_dyna_degrees3 = calllib('dynamixel','dxl_read_word', 3, 30);
T3 = (past_dyna_degrees3/axratio)+30;
past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', 4, 30);
T4 = (past_dyna_degrees4/mxratio);


phi3 = T3;%Keep phi3 (height) at same as previous (want to hold onto domino)
phi4 =(phi2+1.5-T2)+(phi1-T1)+T4; %Adjust pose angle so the world pose is unchanged between points
moveMotors([1,2,3,4],[phi1,(phi2+1.5),phi3,phi4]); %Move the Motors (+1.5 in phi2 is for a design offset of  the end effector)
count = count+1; %Check how many coordinates it has moved to in this path
%If the current coordinate is the last one of the path, change the pose of the domino
%so it is vertical with respect to global axis
if count == number_of_coords
    phi4 = (75+(phi2+1.5-180)+(phi1-180)-90);
    if phi4<0
        phi4 = 360+phi4;
    elseif phi4>=360
        phi4 = phi4-360;
    end
    move_single_motor(4,phi4);
end
end
terminateMotors; %Once the path complete, terminate the motors
end