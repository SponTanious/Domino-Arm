% Domino Arm
% This is the main document code that controls all the functions

%% Setup
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);

%% Find centeral frame
image = step(colorDevice);
center = find_centeral_frame(image);

%% Get Background Image
backImage = step(colorDevice);
backImage = backImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
imtool(backImage);

%% Test
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
centerCropped = [0, 850];

DominoInfo = Domino_Detection(foreImage);

Map = GetMap(backImage, foreImage);
Map = imgaussfilt(Map, 3);
Map = im2bw(Map, 0.25);
imshow(Map);

%% Code
 
while 1
    figure;
    hold on;
    %Move the arm to a position that is outside the visible workspace
    %resetArm();
    
    %Take an image of the workspace
    foreImage = step(colorDevice);
    foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
    imshow(foreImage);
    
    %Get current map of objects and dominos in scene
    Map = GetMap(backImage, foreImage);
    Map = imgaussfilt(Map, 3);
    Map = im2bw(Map, 0.25);
    imshow(Map);
    
    %Detect Dominos and store in domino array
    DominoInfo = Domino_Detection(foreImage);

    %Loop through array of dominos and determine which dominos can be moved to their sorted location.
    for i = 1:size(DominoInfo, 2)
        %Only move domino if it is not already at its goal location
        if (~DominoInfo(i).moved)
            goal_L = round(DominoInfo(i).goal_location);
            current_L = round(DominoInfo(i).current_location);
            
            x = [DominoInfo(i).rectangle1(1:2)+[-10,10], DominoInfo(i).rectangle2(3:4)+[10, -10]];
            y = [DominoInfo(i).rectangle1(6:7)-[10,10], DominoInfo(i).rectangle2(8:9)+[10,10]];
            
            x = double([x, x(1)]);
            y = double([y, y(1)]);
            
            current_domino_mask = poly2mask(x, y, 771, 1701);
            Map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
            imshow(Map);
            disp('yay')
            Path = GreedSearch(Map, goal_L, current_L);
            
            %Greed search returns a blank path if there is no possible path to the goal location
            %If path is not empty, command arm to move domino
            if (~isempty(Path))
                
                new_path = [];
                for i = 1:size(Path, 1)
                    x = centerCropped(2) - Path(i, 1);
                    y = Path(i, 2);
                    new_path = [new_path; x, y];
                end

                %Function for moving domino to goal location
                %MoveDomino(Path);
                plot(new_path(:, 2), new_path(:,1));
                

            end
        end
    end
      
    
%     %Create Map
%     Map = GetMap(backImage, colorDevice);
%     Map = imgaussfilt(Map, 3);
%     Map = im2bw(Map, 0.25);
%     
%     %solve for path
%     Path = GreedSearch(Map, startLocation, endLocation);
%     
%     %operate actuator
    
    
end
