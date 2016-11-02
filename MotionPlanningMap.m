%% Save background image
close all;
clear all;
 
colourDevice = imaq.VideoDevice('kinect',1);
step(colourDevice);

backgroundImage = step(colourDevice);

imshow(backgroundImage);

%% Save foregound image
foregroundImage = step(colourDevice);

BinarisedBackground = im2bw(backgroundImage, 0.83);
BinarisedForeground = im2bw(foregroundImage, 0.83);

differenceImage2 = imfuse(BinarisedForeground,BinarisedBackground,'diff', 'Scaling', 'independent');

Binarised = im2bw(differenceImage2);

Size = size(Binarised);

Ones = ones(Size(1), Size(2));
Ones_logical = cast(Ones, 'logical');

Image = Ones_logical - Binarised;

figure(1)
imshow(Image);
title('Difference Function')
% figure(2)
% imshow(BinarisedForeground);
% title('Binarised Function')

