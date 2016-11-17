function move_to_domino(points_lu)
%% Initialisation Stuff
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
z = (pos(3)+2)*0.01-z0;
phi3 = 180+z*1000*degree_per_mm;
move_single_motor(3,phi3);
% move_single_motor(4,0);
%% Find Angles
tic
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
% % Check Point Location & Account For Error
% if phi2>=180
%     zcorrect = (((phi1-180)/90)*0.95+(abs(180-phi2)/130)*1.7);
% else
%     zcorrect = (((180-phi1)/90)*1.3+(abs(180-phi2)/130)*2.5);
% end
% 
% if pos(2)>30
%     zcorrect = zcorrect-0.5;
% elseif pos(2)<=10
%     zcorrect = zcorrect-0.3;
% end
% if pos(1)<=10 && pos(1)>=0
%     zcorrect = zcorrect-0.5;
% elseif pos(1)>=25
%     zcorrect = zcorrect+0.5;
% end
% z = (pos(3)+zcorrect)*0.01-z0;
% phi3 = 180+z*1000*degree_per_mm;

%Positive x
% if pos(1)<= 10 && pos(1)>=0 && pos(2)<=10 && pos(2)>=0 %1st row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+0.4)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+0.8)*0.01-z0;
%     end
% elseif pos(1)<= 20 && pos(1)>10 && pos(2)<=10 && pos(2)>=0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.09*pos(1);
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)-1)*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 30 && pos(1)>20 && pos(2)<=10 && pos(2)>=0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.08*pos(1);
%         xcorrect = 0.02*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 40 && pos(1)>30 && pos(2)<=10 && pos(2)>=0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.065*pos(1);
%         xcorrect = 0.01*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2)-1)*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 10 && pos(1)>=0 && pos(2)<=20 && pos(2)>10 %2nd row in y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.04*pos(1)+0.04*pos(2);
%         xcorrect = 0.01*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2)-0)*0.01-y0;
%         z = (pos(3)+zcorrect+0.5)*0.01-z0;
%     end
% elseif pos(1)<= 20 && pos(1)>10 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.045*pos(1)+0.04*pos(2);
%         xcorrect = 0.025*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2)-0)*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 30 && pos(1)>20 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.030*pos(1)+0.020*pos(2)+1;
%         if pos(1)>=27 && pos(2)>=18
%             zcorrect = zcorrect-0.028*pos(1);
%             
%         end
%         xcorrect = 0.025*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2))*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 40 && pos(1)>30 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.01*pos(1)+0.005*pos(2)+1;
%         if pos(1)>=33 && pos(2)>=17
%             zcorrect = zcorrect-0.5;
%         end
%         xcorrect = 0.025*pos(1);
%         x = (pos(1)+xcorrect)*0.01-x0;
%         y = (pos(2))*0.01-y0;
%         z = (pos(3)+zcorrect)*0.01-z0;
%     end
% elseif pos(1)<= 10 && pos(1)>=0 && pos(2)<=30 && pos(2)>20 %3rd row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         zcorrect = 0.04*pos(1)+0.04*pos(2);
%         xcorrect = 0.01*pos(1);
%         x = (pos(1)+xcorrect+0.5)*0.01-x0;
%         y = (pos(2)-0)*0.01-y0;
%         z = (pos(3)+zcorrect+0.5)*0.01-z0;
%     end
% elseif pos(1)<= 20 && pos(1)>10 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 30 && pos(1)>20 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 40 && pos(1)>30 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 10 && pos(1)>=0 && pos(2)<=40 && pos(2)>30 % 4th row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 20 && pos(1)>10 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 30 && pos(1)>20 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)<= 40 && pos(1)>30 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% %Negative x
% elseif pos(1)>= -10 && pos(1)<=0 && pos(2)<=10 && pos(2)>0 %1st row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -20 && pos(1)<-10 && pos(2)<=10 && pos(2)>0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -30 && pos(1)<-20 && pos(2)<=10 && pos(2)>0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -40 && pos(1)<-30 && pos(2)<=10 && pos(2)>0
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -10 && pos(1)<=0 && pos(2)<=20 && pos(2)>10 %2nd row in y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -20 && pos(1)<-10 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -30 && pos(1)<-20 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -40 && pos(1)<-30 && pos(2)<=20 && pos(2)>10
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -10 && pos(1)<=0 && pos(2)<=30 && pos(2)>20 %3rd row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -20 && pos(1)<-10 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -30 && pos(1)<-20 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -40 && pos(1)<-30 && pos(2)<=30 && pos(2)>20
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -10 && pos(1)<=0 && pos(2)<=40 && pos(2)>30 % 4th row of y
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -20 && pos(1)<-10 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -30 && pos(1)<-20 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% elseif pos(1)>= -40 && pos(1)<-30 && pos(2)<=40 && pos(2)>30
%     if elbow == 1
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     elseif elbow == 2
%         x = (pos(1)+0)*0.01-x0;
%         y = (pos(2)+0)*0.01-y0;
%         z = (pos(3)+3)*0.01-z0;
%     end
% end
% 
% %Calculate rotation in motor A, B & C:
% phi22_1 = atan2( sqrt( 1-(((x)^2+(y)^2-(L2)^2-(L3)^2)/(2*L2*L3))^2 ), (x^2+y^2-(L2)^2-(L3)^2)/(2*(L2)*(L3)) );
% phi22_2 = -atan2( sqrt( 1-(((x)^2+(y)^2-(L2)^2-(L3)^2)/(2*L2*L3))^2 ), (x^2+y^2-(L2)^2-(L3)^2)/(2*(L2)*(L3)) );
% k1_1 = L2 + L3*cos(phi22_1);
% k1_2 = L2 + L3*cos(phi22_2);
% k2_1 = L3*sin(phi22_1);
% k2_2 = L3*sin(phi22_2);
% phi11_1 = atan2(y, x) - atan2(k2_1, k1_1);
% phi11_2 = atan2(y, x) - atan2(k2_2, k1_2);
% 
% phi1_1 = ((phi11_1)*180/pi)+90;
% phi2_1 = ((phi22_1)*180/pi)+180;
% phi1_2 = ((phi11_2)*180/pi)+90;
% phi2_2 = ((phi22_2)*180/pi)+180;
% 
% %% Check We Have The Right Elbow Solution, If not, change solution.
% %Check which of the two solutions are valid
% solution_1_elbow = 0;
% solution_2_elbow = 0;
% elbow=0;
% if 90<phi1_1 && phi1_1<270
%     solution_1 = 1; %1 Means Valid solution
% else
%     solution_1 = 0;
% end
% if 90<phi1_2 && phi1_2<270
%     solution_2 = 1;
% else
%     solution_2 = 0;
% end
% %Check there is at least 1 valid solution (if not, give error)
% if solution_1 == 0 && solution_2 == 0
%     error('Coordinate Not Physically Possible')
% end
% % Check Elbow of Valid Solutions
% if solution_1 == 1
%     if phi2_1>180 && phi1_1<180
%         solution_1_elbow = 1; %1 Means Elbow Right   
%     elseif phi2_1<180 && phi1_1<180
%         solution_1_elbow = 2; %2 Means elbow Left   
%     elseif phi1_1>180 && phi2_1<180
%         solution_1_elbow = 2;   
%     elseif phi1_1>180 && phi2_1>180
%         solution_1_elbow = 1;
%     end
% end
% 
% if solution_2 == 1
%     if phi2_2>180 && phi1_2<180
%         solution_2_elbow = 1; %1 Means Elbow Right   
%     elseif phi2_2<180 && phi1_2<180
%         solution_2_elbow = 2; %2 Means elbow Left   
%     elseif phi1_2>180 && phi2_2<180
%         solution_2_elbow = 2;   
%     elseif phi1_2>180 && phi2_2>180
%         solution_2_elbow = 1;
%     end
% end
% % Check if Previous Solution was elbow left or right
% past_dyna_degrees1 = calllib('dynamixel','dxl_read_word', 1, 30);
% T1 = (past_dyna_degrees1/axratio)+30;
% past_dyna_degrees2 = calllib('dynamixel','dxl_read_word', 2, 30);
% T2 = (past_dyna_degrees2/axratio)+30;
% 
% if T1<180 && T2>180
%     past_solution_elbow = 1;
% elseif T1<180 && T2<180
%     past_solution_elbow = 2;
% elseif T1>180 && T2>180
%     past_solution_elbow = 1;
% elseif T1>180 && T2<180
%     past_solution_elbow = 2;
% end
% 
% % Pick Between Solutions (if more than 1 is valid)
% if past_solution_elbow == 1
%     if solution_1_elbow == 1
%         phi1 = phi1_1;
%         phi2 = phi2_1;
%         elbow=1;
%     elseif solution_2_elbow == 1
%         phi1 = phi1_2;
%         phi2 = phi2_2;
%         elbow=1;
%     else
%         if solution_1 == 1
%             phi1 = phi1_1;
%             phi2 = phi1_2;
%             elbow = solution_1_elbow;
%         else
%             phi1 = phi1_2;
%             phi2 = phi2_2;
%             elbow=solution_2_elbow;
%         end
%     end
% elseif past_solution_elbow == 2
%     if solution_1_elbow == 2
%         phi1 = phi1_1;
%         phi2 = phi2_1;
%         elbow =2;
%     elseif solution_2_elbow == 2
%         phi1 = phi1_2;
%         phi2 = phi2_2;
%         elbow=2;
%     else
%         if solution_1 == 1
%             phi1 = phi1_1;
%             phi2 = phi1_2;
%             elbow = solution_1_elbow;
%         else
%             phi1 = phi1_2;
%             phi2 = phi2_2;
%             elbow=solution_2_elbow;
%         end
%     end
% end
z = (pos(3))*0.01-z0;
phi3 = 180+z*1000*degree_per_mm;
% move_single_motor(3,phi3);
% move_single_motor(4,0);

past_dyna_degrees4 = calllib('dynamixel','dxl_read_word', 4, 30);
T4 = (past_dyna_degrees4/mxratio);
phi4 = T4;
toc
moveMotors([1,2,3,4],[phi1,(phi2+1.5),phi3,phi4]);
%% Drop Onto Point
% z = z-0.01;
% phi3 = 180+z*1000*degree_per_mm;
% move_single_motor(3,phi3);
% 
% phi4 = T4+180;
% move_single_motor(4,phi4);
end