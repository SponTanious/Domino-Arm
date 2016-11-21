function position = find_centeral_frame(image)
%find_centeral_frame finds the centeral frame of the image using face
%detection

position = [];

% Create a cascade detector object.
faceDetector = vision.CascadeObjectDetector('FrontalFaceLBP');

% Run the detector.
bboxes = step(faceDetector, image);

for i = 1:size(bboxes, 1)
    x  = bboxes(i, 1) + bboxes(i, 3)/2;
    y = bboxes(i, 2) + bboxes(i, 4);
    [in,on] = inpolygon(x, y , [800, 1120, 1120, 800, 800], [0, 0, 300, 300, 0]);
    if (in)
        position = [x,y];
        break
    end
end

if (isempty(position))
    error('Centeral Frame not found')
end

% Draw the returned bounding box around the detected face.
videoOut = insertObjectAnnotation(image,'rectangle',bboxes,'Face');
figure, imshow(videoOut), title('Detected face');

end

