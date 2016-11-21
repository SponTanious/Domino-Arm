%% Maze 1
close all;
a = imread('Maze2.PNG');
Map = a;
MAPSIZE1 = size(Map);
Map = imresize(Map, 0.2);
Map = im2bw(Map);
MAPSIZE2 = size(Map);
imtool(Map);

column_ratio = MAPSIZE1(2)/MAPSIZE2(2);
row_ratio = MAPSIZE1(1)/MAPSIZE2(1);

current_L = [112, 112];
goal_L = [8, 55];

%% Maze 2
a = imread('maze2.png');
a = imresize(a, 0.25);
a = im2bw(a);
Map = a;

current_L = [313, 6];
goal_L = [341, 598];

current_L = [80, 3];
goal_L = [87, 151];

%% 14 x 14 Map
Map = [1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0;
 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1];

 current_L = [1, 1];
 goal_L = [14, 14];
 
 %% 5 x 5 Map
 
 Map = [1 1 1 1 1;
  1 1 0 1 1;
  1 0 0 0 1;
  1 1 0 1 1;
  1 1 1 1 1]

current_L = [1 1];
goal_L = [5 5];
 
%% Map with no obstacles
Map = ones(100, 100);
current_L = [50, 50];
goal_L = [75, 75];

 