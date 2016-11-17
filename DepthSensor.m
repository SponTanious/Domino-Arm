%% Setup and Save Blank Workspace

close all;
clear all;
 
depthDevice = imaq.VideoDevice('kinect',2);
step(depthDevice);

depthImage = step(depthDevice);

BlankWorkspace = pcfromkinect(depthDevice,depthImage);
[xyzBlankWorkspace,~] = depthToPointCloud(depthImage,depthDevice);

release(depthDevice);

%% Find new image and compare

close all;
 
% depthDevice = imaq.VideoDevice('kinect',2);
% step(depthDevice);

depthImage = step(depthDevice);

NewWorkspace = pcfromkinect(depthDevice,depthImage);
[xyzNewWorkspace,~] = depthToPointCloud(depthImage,depthDevice);

NewWorkspaceDist = sqrt( (NewWorkspace.Location(:,:,1)).^2 + (NewWorkspace.Location(:,:,2)).^2 + (NewWorkspace.Location(:,:,3)).^2 );
BlankWorkspaceDist = sqrt( (BlankWorkspace.Location(:,:,1)).^2 + (BlankWorkspace.Location(:,:,2)).^2 + (BlankWorkspace.Location(:,:,3)).^2 );

Subtraction = (NewWorkspaceDist - BlankWorkspaceDist);

% BinNewWorkspaceDist = im2bw(Subtraction, 0.001);

release(depthDevice);

figure;
imshow(adapthisteq(Subtraction));
imtool(adapthisteq(Subtraction));
title('New Workspace');
xlabel('X');
ylabel('Y');
zlabel('Z');




%% Plot workspace differences

figure
pcshowpair(NewWorkspace,BlankWorkspace,'VerticalAxis','Y','VerticalAxisDir','Down')
title('Difference Between Two Point Clouds')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z(m)')