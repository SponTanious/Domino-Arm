function Failed = detectCollision(Node, Map, Pose, height, width)
%detectCollision Summary of this function goes here
%   Detailed explanation goes here

Map_Size = size(Map);

% figure;
% imshow(Map);
% hold on;


%Corner points
point1_x = Node(2)-0.5*width*cos(Pose)-0.5*height*sin(Pose);
point1_y = Node(1)+0.5*width*sin(Pose)-0.5*height*cos(Pose);
point2_x = Node(2)+0.5*width*cos(Pose)-0.5*height*sin(Pose);
point2_y = Node(1)-0.5*width*sin(Pose)-0.5*height*cos(Pose);
point3_x = Node(2)+0.5*width*cos(Pose)+0.5*height*sin(Pose);
point3_y = Node(1)-0.5*width*sin(Pose)+0.5*height*cos(Pose);
point4_x = Node(2)-0.5*width*cos(Pose)+0.5*height*sin(Pose);
point4_y = Node(1)+0.5*width*sin(Pose)+0.5*height*cos(Pose);

%Side points
point5_x = Node(2) + 0.5*width*cos(Pose);
point5_y = Node(1) + 0.5*width*sin(Pose);
point6_x = Node(2) + 0.5*height*sin(Pose);
point6_y = Node(1) - 0.5*height*cos(Pose); 
point7_x = Node(2) - 0.5*width*cos(Pose);
point7_y = Node(1) - 0.5*width*sin(Pose);
point8_x = Node(2) - 0.5*height*sin(Pose);
point8_y = Node(1) + 0.5*height*cos(Pose);

%Between corners and side points (quarter length)
point9_x = point8_x + 0.25*width*cos(Pose);
point9_y = point8_y + 0.25*width*sin(Pose);
point10_x = point8_x - 0.25*width*cos(Pose);
point10_y = point8_y - 0.25*width*sin(Pose);
point11_x = point6_x - 0.25*width*cos(Pose);
point11_y = point6_y - 0.25*width*sin(Pose);
point12_x = point6_x + 0.25*width*cos(Pose);
point12_y = point6_y + 0.25*width*sin(Pose);

x1 = [point1_x, point2_x, point3_x, point4_x, point5_x, point6_x, point7_x, point8_x, point9_x, point10_x, point11_x, point12_x];
y1 = [point1_y, point2_y, point3_y, point4_y, point5_y, point6_y, point7_y, point8_y, point9_y, point10_y, point11_y, point12_y];

Edges = [ceil(y1'),ceil(x1')];
edgeSize = size(Edges);
Failed = 0;
MapSize = size(Map);
for i=1:edgeSize(1)
    Point = Edges(i, :);
    if ((Point(1) <= 0) || (Point(1) > MapSize(1)))
        Failed = 1;
        break
    end
    
    if ((Point(2) <= 0) || (Point(2) > MapSize(2)))
        Failed = 1;
        break
    end
    Object = Map(Point(1), Point(2));
    
    if ~Object
        Failed = 1;
        break
    end
end

% plot(y1, x1);

% current_domino_mask = poly2mask(double(x1), double(y1), Map_Size(1), Map_Size(2));
% new_map = current_domino_mask.*imcomplement(Map);
% 
% cropD = round(sqrt(width^2+height^2)/2);
% 
% newmapsize = size(new_map);
% 
% %Set bounds for collision detection
% 
% if ( (Node(1)-cropD) < 0 )
%     a = 0;
% else
%     a = Node(1)-cropD;
% end
% 
% 
% if ( (Node(1)+cropD) > newmapsize(1) )
%     b = newmapsize(1);
% else
%     b = Node(1)+cropD;
% end
% 
% 
% if ( (Node(2) - cropD) < 0 )
%     c = 0;
% else
%     c = Node(2) - cropD;
% end
% 
% 
% if ( (Node(2) + cropD) > newmapsize(2) )
%     d = newmapsize(2);
% else
%     d = Node(2) + cropD;
% end
% 
% 
% objects= find(new_map( a:b, c:d ) ,1);

% 
% if( isempty(objects))
%     Failed = 0;
% else
%     Failed = 1;
% end
% end



% %Create polygon around current pixel
% height = 56;
% width = 110;
% 
% map_crop = imcrop(Map, [Node(2)-width/2, Node(1)-height/2, width, height]);
% 
% for i = 1:size(map_crop, 1)
%     for j = 1:size(map_crop, 2)
%         if (map_crop(i,j) == 0)
%             Failed = 1;
%             return
%         end
%     end
% end
% 
% Failed = 0;

