%% Save background image
close all;
clear all;
 
colourDevice = imaq.VideoDevice('kinect',1);
step(colourDevice);

backgroundImage = step(colourDevice);

imshow(backgroundImage);

%% Save foregound image
foregroundImage = step(colourDevice);

differenceImage2 = imfuse(foregroundImage,backgroundImage,'diff', 'Scaling', 'independent');

Binarised = im2bw(differenceImage2, 0.2);

Size = size(Binarised);

Ones = ones(Size(1), Size(2));
Ones_logical = cast(Ones, 'logical');

Image = Ones_logical - Binarised;

goal_L = [268, 1321];
current_L = [312, 876];

Path = GreedSearch(Image, goal_L, current_L);

figure(1)
imshow(Image);
hold on;
plot(Path(:, 2), Path(:, 1));
title('Difference Function')
% figure(2)
% imshow(BinarisedForeground);
% title('Binarised Function')

