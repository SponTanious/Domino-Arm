% Domino Arm
% This is the main document code that controls all the functions

%% Setup
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);

%% Get Background Image
backImage = step(colorDevice);

%% Code
while 1
    %Detect Dominos
    DominoInfo = Domino_Detection(step(colorDevice));

    %Find which domino to move and where
    %Loop through array of dominos and determine which dominos can be moved
    %to their sorted location.
    
    for i = 1:size(DominoInfo, 1)
        goal_L = DominoInfo(i, :).goal_location;
        current_L = DominoInfo(i, :).current_location;
    end
      
    
    %Create Map
    Map = GetMap(backImage, colorDevice);
    Map = imgaussfilt(Map, 3);
    Map = im2bw(Map, 0.25);
    
    %solve for path
    Path = GreedSearch(Map, startLocation, endLocation);
    
    %operate actuator
    
    
end
