function [ backgroundImage ] = GetBackgroundImage(colourDevice)
%UNTITLED5 This function saves the original background image for
%   comparison with a cluttered image.

step(colourDevice);

backgroundImage = step(colourDevice);

end

