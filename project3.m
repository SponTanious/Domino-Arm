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
imtool(image);

%% Get Background Image
backImage = step(colorDevice);
backImage = backImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
imtool(backImage);

%% Test Domino Detetection Gamma
close all;
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

originalImage = foreImage;
gray = (rgb2gray(originalImage));
gamma = imadjust(gray, [],[], 1);

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
close all;
Map = GetMap(backImage, foreImage);
Map = imgaussfilt(Map, 3);
Map = imresize(Map, 0.2);
Map = im2bw(Map, 0.9);
imtool(Map);

%% Test Domino Detection
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

dinfo = Domino_Detection(foreImage);

% imtool(foreImage);

%% Code
 close all;
 cm_per_pixel = 0.047267;
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
    Map = im2bw(Map, 0.85);
    LargeMapSize = size(Map);
    imshow(Map);
    
    %Detect Dominos and store in domino array
    DominoInfo = Domino_Detection(foreImage);
    
    %Loop through dominos to find the first domino that can be moved.
    for i = 1:size(DominoInfo, 1)
        if (~DominoInfo(i).moved)
            Domino = DominoInfo(i);
            
            %Only move domino if it is not already at its goal location
            x = [Domino.rectangle1(1:2)+[-30,30], Domino.rectangle2(3:4)+[30, -30]];
            y = [Domino.rectangle1(6:7)-[30,30], Domino.rectangle2(8:9)+[30,30]];

            x = double([x, x(1)]);
            y = double([y, y(1)]);

        %             poly_x = [DominoInfo(i).rectangle1(1:2), DominoInfo(i).rectangle2(3:4)];
        %             poly_y = [DominoInfo(i).rectangle1(6:7), DominoInfo(i).rectangle2(8:9)];

            %The mask dimensions will have to change if the map dimensions
            %change
            current_domino_mask = poly2mask(x, y, 771, 1701);
            new_map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
            
            %Resize map
            SmallMap = imresize(new_map, 0.2);
            SmallMapSize = size(SmallMap);
            new_map = SmallMap;
            new_map = im2bw(new_map, 0.9);
            
            %Get ratios of large column/row to small column/row
            column_ratio = LargeMapSize(2)/SmallMapSize(2);
            row_ratio = LargeMapSize(1)/SmallMapSize(1);

            %Get goal and current locations, and scale down by column and
            %row ratios
            goal_L = round(Domino.goal_location);
            current_L = fliplr(round(Domino.current_location));
            %goal_L = [round(goal_L(1)/row_ratio), round(goal_L(2)/column_ratio)];
            goal_L = [127, 37];
            current_L = [round(current_L(1)/row_ratio), round(current_L(2)/column_ratio)];

            %Get path for small image. 
            Path1 = GreedSearch(new_map, goal_L, current_L, Domino.pose);


            %Greed search returns a blank path if there is no possible path to the goal location
            %If path is not empty, command arm to move domino, then break
            %out of the for loop and re-detect the dominos.
            if (~isempty(Path1))
                
                %Convert path to coords with respect to the central frame
                x_Coords = Path1(:, 2) - center(1);
                Coords = [Path1(:, 1), x_Coords];
                
                %Rescale image path and coordinate path back to original map size
                Rows = row_ratio*Path1(:, 1);
                Columns = column_ratio*Path1(:, 2);
                Path1 = [Rows, Columns];
                Rows = row_ratio*Coords(:, 1);
                Columns = column_ratio*Coords(:, 2);
                Path = [Rows, Columns];
                
                close all;
                imshow(foreImage); hold on; plot(Path1(:, 2), Path1(:, 1));
                x = input('Press enter to begin moving the arm');

                %Function for moving domino to goal location
                %MoveDomino(Path);
                Path = Path*mm_per_pixel;
                pause(1);

                break
            end
        end
    end
end

