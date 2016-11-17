clear all;
close all;

depthDevice = imaq.VideoDevice('kinect',2);
step(depthDevice);

depthImage = step(depthDevice);

ptCloud = pcfromkinect(depthDevice,depthImage);

player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
	'VerticalAxis','y','VerticalAxisDir','down');

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');


for i = 1      
   depthImage = step(depthDevice);
 
   ptCloud = pcfromkinect(depthDevice,depthImage);
 
   view(player,ptCloud);
end

release(depthDevice);