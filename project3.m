% Domino Arm
% This is the main document code that controls all the functions

%% Setup
close all;
clear all;

colorDevice = imaq.VideoDevice('kinect',1);

%% Find centeral frame
image = step(colorDevice);
center = find_centeral_frame(image);
centerCropped = [1, 850];

%% Get Background Image
backImage = step(colorDevice);
backImage = backImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
imtool(backImage);

%% Test Domino Detetection Gamma
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

originalImage = foreImage;
gray = (rgb2gray(originalImage));
gamma = imadjust(gray, [],[], 6);

%Detect FEATURES
points = detectMSERFeatures(gamma, 'MaxAreaVariation', 0.25);

%Display first frame WILL NEED TO REMOVE
figure; 
h = imshow(gamma);
hold on;

%% Test Domino Detetection Binary
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

originalImage = foreImage;
gray = (rgb2gray(originalImage));

I = adapthisteq(gray);
I = im2bw(I, 0.93).*255;

figure; 
h = imshow(I);
hold on;

%% Test Get Map Binary
Map = GetMap(backImage, foreImage);
Map = imgaussfilt(Map, 3);
Map = im2bw(Map, 0.25);
imshow(Map);

%% Test Domino Detection
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

dinfo = Domino_Detection(foreImage);
% imtool(foreImage);

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
            
            x = [DominoInfo(i).rectangle1(1:2)+[-30,30], DominoInfo(i).rectangle2(3:4)+[30, -30]];
            y = [DominoInfo(i).rectangle1(6:7)-[30,30], DominoInfo(i).rectangle2(8:9)+[30,30]];
            
            x = double([x, x(1)]);
            y = double([y, y(1)]);
            
%             poly_x = [DominoInfo(i).rectangle1(1:2), DominoInfo(i).rectangle2(3:4)];
%             poly_y = [DominoInfo(i).rectangle1(6:7), DominoInfo(i).rectangle2(8:9)];
            
            current_domino_mask = poly2mask(x, y, 771, 1701);
            new_map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
            imshow(new_map);
            
            Path = GreedSearch(new_map, goal_L, current_L, DominoInfo(i).pose);
            
            %Greed search returns a blank path if there is no possible path to the goal location
            %If path is not empty, command arm to move domino
            if (~isempty(Path))
                
                new_path = [];
                for i = 1:size(Path, 1)
                    x = (centerCropped(2) - Path(i, 1));
                    y = Path(i, 2);
                    new_path = [new_path; x, y];
                end
                
                %Function for moving domino to goal location
                %MoveDomino(Path);
                plot(Path(:, 2), Path(:,1));
                pause(1);
                
                new_path = new_path*21.23;
                break

            end
        end
    end
    
    
end
