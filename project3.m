% Domino Arm
% This is the main document code that controls all the functions

%% Setup
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);
colourDevice.FramesPerTrigger = 10000;




%% Object Mapping

point1 = [760, 481];
point2 = [2249, 497];
point3 = [2219, 1394];
point4 = [769, 1380];

xleft = mean([point1(1), point4(1)]);
xright = mean([point2(1), point3(1)]);
ytop = mean([point1(2), point2(2)]);
ybottom = mean([point3(2), point4(2)]);

width = 320;
height = 192;

mmpx = 0.41;

boxtoframe = 10;

a = size(NoDuplicates);
for i = 1:a(1)s
    pos = NoDuplicates(i, 2:3);
    CtoD_x = width/2 -(pos(1)-xleft)/(xright-xleft)*width;
    CtoD_y = -1 * ((pos(2)-ybottom)/(ytop-ybottom)*height + boxtoframe );
    CtoD_z = -1*locationOtoC(3);
    CtoD = [CtoD_x; CtoD_y; CtoD_z];
    OtoD = locationOtoC + CtoD
    %dist = sqrt(OtoD(1)^2+OtoD(2)^2+OtoD^(3))
end