%% Setup and Save Blank Workspace
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);
colourDevice.FramesPerTrigger = 10000;

depthDevice = imaq.VideoDevice('kinect', 2);


BlankWorkspace = step(depthDevice);





%% Take New Image and Compare with blank

depthImage = step(depthDevice);

Subtracted = 50*(depthImage - BlankWorkspace);

xyzPoints = depthToPointCloud(Subtracted,depthDevice);

pcshow(xyzPoints,'VerticalAxis','y','VerticalAxisDir','down');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

release(depthDevice);

