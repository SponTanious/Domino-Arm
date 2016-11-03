function [dominoArray] = Domino_Detection(image)
%Domino_Detection takes an image and find the loactions

dominoArray = [];

%Manipulate Image
originalImage = image;
originalImage = originalImage(100:840, 640:1500, :);
gray = (rgb2gray(originalImage));
gamma = imadjust(gray, [],[], );

%Detect FEATURES
points = detectMSERFeatures(gamma, 'MaxAreaVariation', 0.25)

%Display first frame
figure; 
h = imshow(gamma);
hold on;

%Binarize Image and find circles
%differenceImage = imfuse(originalImage, emptyImage(100:840, 640:1500, :), 'diff', 'Scaling', 'independent');
I = adapthisteq(gray);
I = im2bw(I, 0.95).*255;
[centers,radii] = imfindcircles(I, [4, 5],'ObjectPolarity','dark','Sensitivity',0.94,'EdgeThreshold',0.1, 'Method', 'twostage');

%Loop through features
a = size(points);
LineInfo = [];
serfCenters = [];
serfRadii = [];
for n = 1:a(1)
    axe = points(n).Axes;
    pos = points(n).Location;
    orient = points(n).Orientation;
    if((axe(1) > 5*axe(2)))
        if((axe(1) < 55) & (axe(1) > 35) & (axe(2) < 20) & (axe(2) > 1))
            if( gamma(round(pos(2)), round(pos(1))) <= 5)
                LineInfo = [LineInfo; n pos orient axe];
            end
        end
    end
    if( (axe(1) <= 2*axe(2)) )
        if (axe(1)< 30) 
             serfCenters = [serfCenters; pos];
             serfRadii = [serfRadii; mean(axe)/2];
        end
    end
end

%prevent duplicates
a = size(LineInfo);
NoDuplicates = [];
DuplicateArray = [];

for i = 1:(a(1)-1)
    dup = 1;
    b = size(DuplicateArray);
    for j = 1:b(1)
        %Check if exist in Duplicate array
        if(DuplicateArray(j) == LineInfo(i))
            dup = 0;
            break
        end
    end
    if(dup == 1)
        for k = (i+1):a(1)
            %Check distance is less than axes
            pos1 = LineInfo(i, 2:3);
            pos2 = LineInfo(k, 2:3);
            vect = pos2-pos1;
            dist =sqrt(vect(1)^2+vect(2)^2);
            if(dist <= 30)
                DuplicateArray = [DuplicateArray; LineInfo(k,:)];
            end
        end
        NoDuplicates = [NoDuplicates; LineInfo(i, :)];
    end
end

% Draw lines
a = size(NoDuplicates);
for i = 1:a(1)
    n = NoDuplicates(i, 1);
    plot(points(n), 'showPixelList', false, 'showEllipses', true);
end

% plot(points, 'showPixelList', false, 'showEllipses', true);

allCenters = [centers; serfCenters];
allRadii = [radii; serfRadii];

a = size(allCenters);
b = size(allRadii);
NoDuplicatesC = [];
NoDuplicatesR = [];
DuplicateArrayC = [];
DuplicateArrayR = [];

for i = 1:(a(1)-1)
    dup = 1;
    b = size(DuplicateArrayC);
    for j = 1:b(1)
        %Check if exist in Duplicate array
        if(DuplicateArrayC(j) == allCenters(i))
            dup = 0;
            break
        end
    end
    if(dup == 1)
        for k = (i+1):a(1)
            %Check distance is less than axes
            pos1 = allCenters(i, 1:2);
            pos2 = allCenters(k, 1:2);
            vect = pos2-pos1;
            dist =sqrt(vect(1)^2+vect(2)^2);
            if(dist <= 10)
                DuplicateArrayC = [DuplicateArrayC; allCenters(k,:)];
                DuplicateArrayR = [DuplicateArrayR; allRadii(k,:)];
            end
        end
        NoDuplicatesC = [NoDuplicatesC; allCenters(i, :)];
        NoDuplicatesR = [NoDuplicatesR; allRadii(i, :)];
    end
end

%Draw Circles
viscircles(NoDuplicatesC, NoDuplicatesR,'EdgeColor','r');

%Draw Rectangles
a = size(NoDuplicates);
RectangleInfo = [];
for n = 1:a(1)
  B = NoDuplicates(n,:);
  height = 2*B(5);
  width = B(5)*1;
  point1_x = B(2)-0.5*width*cos(B(4))-0.5*height*sin(B(4));
  point1_y = B(3)+0.5*width*sin(B(4))-0.5*height*cos(B(4));
  point2_x = B(2)+0.5*width*cos(B(4))-0.5*height*sin(B(4));
  point2_y = B(3)-0.5*width*sin(B(4))-0.5*height*cos(B(4)); 
  point3_x = B(2)+0.5*width*cos(B(4))-0.05*height*sin(B(4));
  point3_y = B(3)-0.5*width*sin(B(4))-0.05*height*cos(B(4));
  point4_x = B(2)-0.5*width*cos(B(4))-0.05*height*sin(B(4));
  point4_y = B(3)+0.5*width*sin(B(4))-0.05*height*cos(B(4));
  
  point5_x = B(2)-0.5*width*cos(B(4))+0.05*height*sin(B(4));
  point5_y = B(3)+0.5*width*sin(B(4))+0.05*height*cos(B(4));
  point6_x = B(2)+0.5*width*cos(B(4))+0.05*height*sin(B(4));
  point6_y = B(3)-0.5*width*sin(B(4))+0.05*height*cos(B(4));
  point7_x = B(2)+0.5*width*cos(B(4))+0.5*height*sin(B(4));
  point7_y = B(3)-0.5*width*sin(B(4))+0.5*height*cos(B(4));
  point8_x = B(2)-0.5*width*cos(B(4))+0.5*height*sin(B(4));
  point8_y = B(3)+0.5*width*sin(B(4))+0.5*height*cos(B(4));

  x1 = [point1_x, point2_x, point3_x, point4_x, point1_x];
  y1 = [point1_y, point2_y, point3_y, point4_y, point1_y];
  
  x2 = [point5_x, point6_x, point7_x, point8_x, point5_x];
  y2 = [point5_y, point6_y, point7_y, point8_y, point5_y];
  
  RectangleInfo = [RectangleInfo; x1, y1, x2, y2];
  
  plot(x1, y1)
  plot(x2, y2)
end

for k = 1:size(RectangleInfo, 1)
    value1 = 0;
    circle1 = [];
    for i = 1:size(NoDuplicatesC)
        [in,on] = inpolygon(NoDuplicatesC(i, 1),NoDuplicatesC(i, 2),RectangleInfo(k, 1:5),RectangleInfo(k, 6:10));
        if ((in) || (on))
            value1 = value1 + 1;
            circle1 = [circle1; NoDuplicatesC(i,1:2), NoDuplicatesR(i,:)];
        end
    end
    text(double(RectangleInfo(k, 1)),double(RectangleInfo(k, 6)),sprintf('%d', value1), 'Color', 'blue', 'FontSize', 14);
    value2 = 0;
    circle2 = [];
    for i = 1:size(NoDuplicatesC)
        in = inpolygon(NoDuplicatesC(i, 1),NoDuplicatesC(i, 2),RectangleInfo(k, 11:15),RectangleInfo(k, 16:20));
        if (in)
            value2 = value2 + 1;
            circle2 = [circle2; NoDuplicatesC(i,:), NoDuplicatesR(i, :)];
        end
    end
    text(double(RectangleInfo(k, 11)),double(RectangleInfo(k, 16)),sprintf('%d', value2), 'Color', 'blue', 'FontSize', 14);
    
    dominoArray = [dominoArray, Domino([value1, value2], RectangleInfo(k, 1:10), circle1, RectangleInfo(k, 11:20), circle2)];
end

end

