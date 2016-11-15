function move_with_domino(points_lu)
load('ArmVariables.mat');

points_lu = transpose(points_lu);
initMotors;
terminateMotors;
initMotors;
%% Move with domino between a series of coordinates
for pos = points_lu
%Define Goal Pos:
x = pos(1)*0.01-x0;
y = pos(2)*0.01-y0;
z = (pos(3)+2)*0.01-z0;

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

%% Check We Have The Right Elbow Solution, if not, Change Solution.
if 90<phi1_1 && phi1_1<270
    phi1 = phi1_1;
    phi2 = phi2_1;
    
else
    phi1 = phi1_2;
    phi2 = phi2_2;
end
% Check Elbow for Coordinates of Two Valid Solution
if phi2>180 && phi1<180
    solution = 1; %1 Means Elbow down    
elseif phi2<180 && phi1<180
    solution = 2; %2 Means elbow up   
elseif phi1>180 && phi2<180
    solution = 1;   
else
    solution = 2;
end
%Check we can reach solution
if phi1>270
    error('Coordinate Not Physically Possible')
end
if phi1<90
    error('Coordinate Not Physically Possible')
end

%% Finalize Angles & Move Motors
past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', 1, 30);
T1 = (past_dyna_degrees1/axratio)+30;
past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', 2, 30);
T2 = (past_dyna_degrees2/axratio)+30;
past_dyna_degrees3 = calllib('dynamixel','dxl_read_word', 3, 30);
T3 = (past_dyna_degrees3/axratio)+30;
past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', 4, 30);
T4 = (past_dyna_degrees4/mxratio);

delta_phi4 = T2-phi2+T1-phi1;
phi4 = T4-delta_phi4;
phi3 = 180+z*1000*degree_per_mm;
moveMotors([1,2,3,4],[phi1,phi2,phi3,phi4]);
end
end
%%terminateMotors;