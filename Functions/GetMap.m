function Map = GetMap( backgroundImage, colourDevice )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

foregroundImage = step(colourDevice);

differenceImage2 = imfuse(foregroundImage,backgroundImage,'diff', 'Scaling', 'independent');


Binarised = im2bw(differenceImage2, 0.2);

imshow(Binarised);

Size = size(Binarised);

Ones = ones(Size(1), Size(2));
Ones_logical = cast(Ones, 'logical');

Map = Ones_logical - Binarised;

end

