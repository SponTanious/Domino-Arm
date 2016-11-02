%Postion of z rotation axis
x0 = 0;
y0 = -0.1;
z0 = 0.087;

%angles
T1 = 180;
T2 = 180;
T3 = 180;

%Arm Lengths
L1 = 0;  
L2 = 0.210;
L3 = 0.211;

%%
points_lu=[[4,0,0]];
points_lu = transpose(points_lu);

for pos = points_lu
    x = pos(1)*0.01;
    y = pos(2)*0.01;
    z = pos(3)*0.01;

    %Calculate rotation in motor A
    if (x0 < x)
        phi1 = 90 + atan((y-y0)/(x-x0))*180/pi;
    elseif (x0 == x)
        phi1 = 180;
    else
        phi1 = 270 - atan((y-y0)/(x0-x))*180/pi;
    end
    Motorpos1 = phi1;
    T1 = phi1;
    phi1
    %Calculate rotation in motor B and C
    %knock off domino
    tx = sqrt((x-x0)^2+(y-y0)^2);
    ty = z;

    
    phi33 = atan2( sqrt( 1-((tx^2+ty^2-L2^2-L3^2)/(2*L2*L3))^2 ), (tx^2+ty^2-L2^2-L3^2)/(2*L2*L3) );
    k1 = L2 + L3*cos(phi33);
    k2 = L3*sin(phi33);
    phi23 = atan2(ty, tx) - atan2(k2, k1);
    if (phi23>0)
        phi2 = (phi23)*180/pi+90
    else
        phi2 = (-phi23-pi)*180/pi+90
    end
    phi3 = -phi33*180/pi+180
    
    T3 = phi3;
    T2 = phi2;
    T1 = phi1;
    
end