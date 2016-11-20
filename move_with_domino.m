function move_with_domino(points_lu)
%Initialisation Stuff
load('ArmVariables.mat');
number_of_coords = size(points_lu);
number_of_coords = number_of_coords(1);
points_lu = transpose(points_lu);
count = 0;
initMotors;
%% Move with domino between a series of coordinates
for pos = points_lu
% Define Goal Pos:
x = pos(1)*0.01-x0;
y = pos(2)*0.01-y0;
z = (pos(3)-0.5)*0.01-z0;
phi3 = 180+z*1000*degree_per_mm;
%% Find Angles
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
elbow=0;
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
        elbow=1;
    elseif solution_2_elbow == 1
        phi1 = phi1_2;
        phi2 = phi2_2;
        elbow=1;
    else
        if solution_1 == 1
            phi1 = phi1_1;
            phi2 = phi1_2;
            elbow = solution_1_elbow;
        else
            phi1 = phi1_2;
            phi2 = phi2_2;
            elbow=solution_2_elbow;
        end
    end
elseif past_solution_elbow == 2
    if solution_1_elbow == 2
        phi1 = phi1_1;
        phi2 = phi2_1;
        elbow =2;
    elseif solution_2_elbow == 2
        phi1 = phi1_2;
        phi2 = phi2_2;
        elbow=2;
    else
        if solution_1 == 1
            phi1 = phi1_1;
            phi2 = phi1_2;
            elbow = solution_1_elbow;
        else
            phi1 = phi1_2;
            phi2 = phi2_2;
            elbow=solution_2_elbow;
        end
    end
end
%% Error Correction Stuff
%Include height compensators here if necessary
%% Finalize Angles & Move Motors
past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', 1, 30);
T1 = (past_dyna_degrees1/axratio)+30;
past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', 2, 30);
T2 = (past_dyna_degrees2/axratio)+30;
past_dyna_degrees3 = calllib('dynamixel','dxl_read_word', 3, 30);
T3 = (past_dyna_degrees3/axratio)+30;
past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', 4, 30);
T4 = (past_dyna_degrees4/mxratio);

% delta_phi4 = T2-phi2+T1-phi1;
% phi4 = T4-delta_phi4;
% phi3 = 180+z*1000*degree_per_mm;
phi3 = T3;
phi4 =(phi2+1.5-T2)+(phi1-T1)+T4;
moveMotors([1,2,3,4],[phi1,(phi2+1.5),phi3,phi4]);
count = count+1;
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
terminateMotors;
end