% Domino Arm
% This is the main document code that controls all the functions

%% Setup
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);
colourDevice.FramesPerTrigger = 10000;
