classdef Domino
    %Domino Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        value
        rectangles
        circles
    end
    
    methods
        function obj = Domino(DominoValue, RectangleInfo, CircleInfo)
            if ((size(DominoValue, 1) == 1) && (size(DominoValue, 2) == 2))
                obj.value = DominoValue;
            else
                error('rectCorners must be a 1 by 2 Vector')
            end
            
            if (size(RectangleInfo, 2) == 10)
                obj.rectangles = RectangleInfo;
            else
                error('rectCorners must be a M by 10 Matrix')
            end
            
            if (size(CircleInfo, 2) == 3)
                obj.circles = CircleInfo;
            else
                error('rectCorners must be a M by 3 Matrix')
            end
          
       end
    end
    
end

