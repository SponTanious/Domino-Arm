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
% imtool(image);

%% Get Background Image
backImage = step(colorDevice);
backImage = backImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);
imtool(backImage);

%% Boundary
bound_x = [0; 1701; 1701; 1661; 1604; 1529; 1410; 1298; 1194; 1126; 1040; 957; 878; 850; 767; 689; 608; 526; 431; 320; 225; 141; 80; 0; 0];
diff = 80;
bound_y = [-diff; -diff; 300; 349; 403; 460; 536; 582; 618; 636; 667; 684; 686; 685; 684; 689; 670; 648; 610; 553; 504; 437; 374; 300; -diff]+diff;

bound_mask = poly2mask(bound_x, bound_y, 771, 1701);

%% Test Domino Detetection Gamma
close all;
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

originalImage = foreImage;
gray = (rgb2gray(originalImage));
gamma = imadjust(gray, [],[], 5);
%Detect FEATURES
points = detectMSERFeatures(gamma, 'MaxAreaVariation', 0.25);

%Display first frame WILL NEED TO REMOVE
figure; 
h = imshow(gamma);
hold on;

plot(points, 'showPixelList', false, 'showEllipses', true);

%% Test Domino Detetection Binary
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

originalImage = foreImage;
gray = (rgb2gray(originalImage));
gamma2 = imadjust(gray, [],[], 7);
I = adapthisteq(gamma2);
I = im2bw(I, 0.8).*255;

[centers,radii] = imfindcircles(I, [4, 7],'ObjectPolarity','dark','Sensitivity',0.9,'EdgeThreshold',0.1, 'Method', 'twostage');

figure; 
h = imshow(I);
hold on;

viscircles(centers, radii,'EdgeColor','r');

%% Test Get Map Binary
close all;
foreImage = step(colorDevice);
foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

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
cm_per_pixel_Rows = 0.047267;
 
while 1
    %Move the arm to a position that is outside the visible workspace
    move_arm_out_of_view;
    
    %Take an image of the workspace
    foreImage = step(colorDevice);
    foreImage = foreImage(center(2):center(2)+770, center(1)-850:center(1)+850, :);

    %Get current map of objects and dominos in scene
    Map = GetMap(backImage, foreImage);
    Map = imgaussfilt(Map, 3);
    Map = im2bw(Map, 0.96).*bound_mask;
    LargeMapSize = size(Map);

    %Detect Dominos and store in domino array
    DominoInfo = Domino_Detection(foreImage);
    
    %Loop through dominos to find the first domino that can be moved.
    for i = 1:size(DominoInfo, 2)
        disp(i);
        if (~DominoInfo(i).moved)
            Domino = DominoInfo(i);
            
            figure
            imshow(Map)
            
            %Only move domino if it is not already at its goal location
            width = 80;
            height = 150;
            point1_x = Domino.current_location(1)-0.5*width*cos(Domino.pose)-0.5*height*sin(Domino.pose);
            point1_y = Domino.current_location(2)+0.5*width*sin(Domino.pose)-0.5*height*cos(Domino.pose);
            point2_x = Domino.current_location(1)+0.5*width*cos(Domino.pose)-0.5*height*sin(Domino.pose);
            point2_y = Domino.current_location(2)-0.5*width*sin(Domino.pose)-0.5*height*cos(Domino.pose);
            point3_x = Domino.current_location(1)+0.5*width*cos(Domino.pose)+0.5*height*sin(Domino.pose);
            point3_y = Domino.current_location(2)-0.5*width*sin(Domino.pose)+0.5*height*cos(Domino.pose);
            point4_x = Domino.current_location(1)-0.5*width*cos(Domino.pose)+0.5*height*sin(Domino.pose);
            point4_y = Domino.current_location(2)+0.5*width*sin(Domino.pose)+0.5*height*cos(Domino.pose);
            
            x = double([point1_x, point2_x, point3_x, point4_x, point1_x]);
            y = double([point1_y, point2_y, point3_y, point4_y, point1_y]);

            %The mask dimensions will have to change if the map dimensions
            %change
            current_domino_mask = poly2mask(x, y, 771, 1701);
            new_map = imcomplement(imcomplement(Map).*imcomplement(current_domino_mask));
            
            figure
            imshow(new_map)
            
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
            Biggoal_L = round(Domino.goal_location);
            Bigcurrent_L = fliplr(round(Domino.current_location));
<<<<<<< HEAD
            goal_L = [round(Biggoal_L(1)/row_ratio), round(Biggoal_L(2)/column_ratio)];
            current_L = [round(Bigcurrent_L(1)/row_ratio), round(Bigcurrent_L(2)/column_ratio)];

            %Get path for small image. 
=======
            goal_L = [round(Biggoal_L(1)/row_ratio), round(Biggoal_L(2)/column_ratio)]
            current_L = [round(Bigcurrent_L(1)/row_ratio), round(Bigcurrent_L(2)/column_ratio)]
            
            %Get path for small image.
>>>>>>> refs/remotes/SponTanious/master
            Path1 = GreedSearch(new_map, goal_L, current_L, Domino.pose);
            disp(Path1);


            %Greed search returns a blank path if there is no possible path to the goal location
            %If path is not empty, command arm to move domino, then break
            %out of the for loop and re-detect the dominos.
            if (~isempty(Path1))
                
                %Convert path to coords with respect to the central frame
                x_Coords = Path1(:, 2);
                Coords = [Path1(:, 1), x_Coords];
                
                %Rescale image path and coordinate path back to original map size
                Rows = row_ratio*Path1(:, 1);
                Columns = column_ratio*Path1(:, 2);
                Path1 = [Rows, Columns];
                
                Rows = row_ratio*Coords(:, 1);
                Columns = column_ratio*Coords(:, 2) - center(1)+70;
                Path = [Rows, Columns];
                
                size = size(Path);
                Path(1, 1) = Bigcurrent_L(1);
                Path(1, 2) = Bigcurrent_L(2);
                Path(size(1), 1) = Biggoal_L(1);
                Path(size(1), 2) = Biggoal_L(2);
                
                close all;
                imshow(foreImage); hold on; plot(Path1(:, 2), Path1(:, 1));

                %Function for moving domino to goal location
                %MoveDomino(Path);
<<<<<<< HEAD
                Path = Path*cm_per_pixel;
                size = size(Path);
                size  = size(1);
                x_coord = Path(:,2);
                y_coord = Path(:,1);
                z_coord = zeros(size, 1);
                path_vector = [x_coord,y_coord,z_coord];
                domino_coord = [x_coord(1),y_coord(1),z_coord(1),Domino.pose*180/pi];
=======
                Path2 = Path*cm_per_pixel_Rows;
                NewPath = Path2(1:15:end, :);
                NewPath = [NewPath(:, 1), NewPath(:, 2); ([0, - center(1)+70]+Biggoal_L)*cm_per_pixel_Rows];
                Msize = size(NewPath);
                Msize  = Msize(1);
                x_coord = NewPath(:,2);
                y_coord = NewPath(:,1);
                z_coord = zeros(Msize, 1);
                path_vector = [x_coord,y_coord,z_coord];
                domino_coord = [x_coord(1),y_coord(1),z_coord(1),(Domino.pose*180/pi)-90];
>>>>>>> refs/remotes/SponTanious/master
                move_to_domino(domino_coord);
                move_with_domino(path_vector);
                break
            end
        end
    end
end

