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

%% Test 1
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
centerCropped = [1, 850];

DominoInfo = Domino_Detection(foreImage);

%% Test 2
Map = GetMap(backImage, foreImage);
Map = imgaussfilt(Map, 3);
Map = im2bw(Map, 0.25);
imshow(Map);

pause(1)

d = 1;

x = [DominoInfo(d).rectangle1(1:2)+[-15,15], DominoInfo(d).rectangle2(3:4)+[15, -15]];
y = [DominoInfo(d).rectangle1(6:7)-[15,15], DominoInfo(d).rectangle2(8:9)+[15,15]];

x = double([x, x(1)]);
y = double([y, y(1)]);

current_domino_mask = poly2mask(x, y, 771, 1701);
Map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
imshow(Map);


goal_L = fliplr( round(DominoInfo(d).goal_location) )
current_L = fliplr( round(DominoInfo(d).current_location) )



Path = GreedSearch(Map, goal_L, current_L);

hold on;

plot(Path(:, 2), Path(:,1));
%% Code
 
while 1
    figure;
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
            goal_L = fliplr(round(DominoInfo(i).goal_location));
            current_L = fliplr(round(DominoInfo(i).current_location));
            
            x = [DominoInfo(i).rectangle1(1:2)+[-15,15], DominoInfo(i).rectangle2(3:4)+[15, -15]];
            y = [DominoInfo(i).rectangle1(6:7)-[15,15], DominoInfo(i).rectangle2(8:9)+[15,15]];
            
            x = double([x, x(1)]);
            y = double([y, y(1)]);
            
            current_domino_mask = poly2mask(x, y, 771, 1701);
            Map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
            imshow(Map);
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
                plot(Path(:, 2), Path(:,1));
                pause(5);

            end
        end
    end
    
    
end
