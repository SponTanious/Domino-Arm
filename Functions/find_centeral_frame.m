function [ output_args ] = find_centeral_frame( input_args )
%find_centeral_frame Summary of this function goes here
%   Detailed explanation goes here

%% Find Central Frame
x = input('Press enter to find central frame: ');

a = size(params.RotationMatrices);
views = a(3);

locationCtoO = params.TranslationVectors(views, :);
rotationCtoO = params.RotationMatrices(:,:,views);

dist1 = sqrt(locationCtoO(1)^2+locationCtoO(2)^2+locationCtoO(3)^2)

locationOtoC = -1*rotationCtoO*transpose(locationCtoO)
rotationOtoC = transpose(rotationCtoO);

alpha = atan2(rotationOtoC(2,1), rotationOtoC(1,1))*180/pi
beta = atan2(rotationOtoC(3,1), sqrt(rotationOtoC(3,2)^2+rotationOtoC(3,3)^2))*180/pi
gamma = atan2(rotationOtoC(3,2), rotationOtoC(3,3))*180/pi

K = transpose(params.IntrinsicMatrix);
E = [rotationOtoC, locationOtoC];

end

