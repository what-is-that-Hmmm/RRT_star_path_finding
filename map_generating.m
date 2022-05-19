%% Initialisation
%   in this file, a png map would be generated, a point with value of 255
%   would be considered as free, with value other than 255 is occupied. 
clear
clc

%% Map details
Xsize = 2400;
Ysize = 3000;
map_name = 'usr_map1.png';

%   The left top corner of an obstacle is its origin

%   obstacle 1
origin_o1_X = 800;
origin_o1_Y = 1800;
shape_o1_X = 500;
shape_o1_Y = 300;

%   obstacle 2
origin_o2_X = 1800;
origin_o2_Y = 200;
shape_o2_X = 300;
shape_o2_Y = 500;

%   obstacle 3
origin_o3_X = 1800;
origin_o3_Y = 2200;
shape_o3_X = 300;
shape_o3_Y = 500;


%   obstacle 4
origin_o4_X = 800;
origin_o4_Y = 1000;
shape_o4_X = 900;
shape_o4_Y = 500;

%%  Creating map
usr_map = ones(Xsize,Ysize)*255;
% add obstacle 1 to the map
for ix = 0:shape_o1_X
   for iy = 0:shape_o1_Y
      usr_map(origin_o1_X+ix, origin_o1_Y+iy)=0; 
   end
end

% add obstacle 2 to the map
for ix = 0:shape_o2_X
   for iy = 0:shape_o2_Y
      usr_map(origin_o2_X+ix, origin_o2_Y+iy)=0; 
   end
end

% add obstacle 3 to the map
for ix = 0:shape_o3_X
   for iy = 0:shape_o3_Y
      usr_map(origin_o3_X+ix, origin_o3_Y+iy)=0; 
   end
end

% add obstacle 3 to the map
for ix = 0:shape_o4_X
   for iy = 0:shape_o4_Y
      usr_map(origin_o4_X+ix, origin_o4_Y+iy)=0; 
   end
end

imshow(usr_map)
imwrite(usr_map,'usr_map1.png');
