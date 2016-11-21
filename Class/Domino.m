classdef Domino
    %Domino is a class that manages the information for each domino.
    
    properties
        value               % Stores the value of the domino in a 1x2 vector
        current_location    % Stores the current location of the domino in a 1x2 vector
        goal_location       % Stores the goal location of the domino in a 1x2 vector
        rectangle1          % Stores the first rectangle polygon vector
        circle1             % Stores the location and radius of the circles found in in rectangle1
        rectangle2          % Stores the second rectangle polygon vector
        circle2             % Stores the location and radius of the circles found in in rectangle2
        moved               % Returns 1 when domino is at goal location, otherwise 0
        pose                % Stores the current pose of the domino
    end
    
    methods
        function obj = Domino(DominoValue, RectangleInfo1, CircleInfo1, RectangleInfo2, CircleInfo2, Pose)
            % Domino constructor
            
            if ((size(DominoValue, 1) == 1) && (size(DominoValue, 2) == 2))
                obj.value = sort(DominoValue);
            else
                error('DominoValue must be a 1 by 2 Vector')
            end
            
            if (size(RectangleInfo1, 2) == 10)
                obj.rectangle1 = RectangleInfo1;
            else
                error('rectCorners1 must be a 1 by 10 Matrix')
            end
            
            if (size(CircleInfo1, 2) <= 3)
                obj.circle1 = CircleInfo1;
            else
                error('CircleInfo1 must be a N by 3 Matrix')
            end
            
            if (size(RectangleInfo2, 2) == 10)
                obj.rectangle2 = RectangleInfo2;
            else
                error('rectCorners2 must be a 1 by 10 Matrix')
            end
            
            if (size(CircleInfo2, 2) <= 3)
                obj.circle2 = CircleInfo2;
            else
                error('CircleInfo2 must be a N by 3 Matrix')
            end
            
            obj.current_location = [0.5*(obj.rectangle1(1)+obj.rectangle2(3)),   0.5*(obj.rectangle1(6)+obj.rectangle2(8))];
            
            obj.goal_location = get_domino_location(obj.value);
            
            vect = (obj.current_location - fliplr(obj.goal_location)).^2;
            if ( sqrt(sum(vect)) <= 100 )
                obj.moved = 1;
            else
                obj.moved = 0;
            end
            
            obj.pose = Pose;
       end
    end
    
end

