close all;
clear all;
 
colourDevice = imaq.VideoDevice('kinect',1);

backgroundImage = GetBackgroundImage(colourDevice);

imshow(backgroundImage);

b = input('Press enter to obtain map');

Map = GetMap(backgroundImage, colourDevice);

imtool(Map);

x = [300, 1500];
y = [100, 100];

Path = GreedSearch(Map, x, y);

figure(1)
imshow(Map);
hold on;
plot(Path(:, 2), Path(:, 1));
title('Difference Function')
