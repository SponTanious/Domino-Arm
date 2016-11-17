function Map = GetMap( backgroundImage, foregroundImage )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

differenceImage2 = imfuse(foregroundImage,backgroundImage,'diff', 'Scaling', 'independent');

Binarised = im2bw(differenceImage2, 0.3);

Size = size(Binarised);

Ones = ones(Size(1), Size(2));
Ones_logical = cast(Ones, 'logical');

Map = Ones_logical - Binarised;

end

