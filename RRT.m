%% Initialise process
clc
clear all; close all;

x_Start=1; y_Start=1;           % set starting point
x_Goal=100; y_Goal=100;       % set target point
Thr=15;                 % target threshold (vicinity to be considered as target)
Delta= 30;              % step length
iteration_num = 3000;   % the iteration number for expanding the tree
greedy_index = 0.1;     % how greedy the tree would expand towards target, 1 for the most greedy

%% Build up initial tree
T.v(1).x = x_Start;         % 'T' as tree, 'v' as node, adding starting point to the tree.
T.v(1).y = y_Start; 
T.v(1).xPrev = x_Start;     % initial parent node is it self.
T.v(1).yPrev = y_Start;
T.v(1).dist=0;          % Euclidean distance from parten not to current node.
T.v(1).indPrev = 0;     
%% building the tree
figure(1);
ImpRgb=imread('usr_map1.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);     %length of map in X
yL=size(Imp,1);     %length of map in Y
hold on
plot(x_Start, y_Start, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
plot(x_Goal, y_Goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');% plot start and target point
count=1;
bFind = false;

for iter = 1:iteration_num
    %Step 1: Randomly sample on the map
    x_rand=[];
    if (rand > greedy_index)     % randomly sample in the map
        x_temp = floor(rand*800);
        y_temp = floor(rand*800);
        x_rand = [x_temp, y_temp];
    else
        x_rand = [x_Goal, y_Goal];
    end
        
    %Step 2: find the nearest node in the tree to 'x_rand'
    x_near=[];
    min_distance = sqrt((x_rand(1)-T.v(1).x)^2 + (x_rand(2)-T.v(1).y)^2);
    min_index_temp = 1;
    if (iter >= 3)
        for i1=2:size(T.v,2)
            temp_distance = sqrt((x_rand(1)-T.v(i1).x)^2 + (x_rand(2)-T.v(i1).y)^2);
            if (temp_distance<min_distance)
                min_distance = temp_distance;
                min_index_temp = i1;
            end     % end of the if for comparing the smaller value
        end     % end of the for loop for findin the nearest node
    end     % end of the if for judging 'iter'
    x_near = [T.v(min_index_temp).x, T.v(min_index_temp).y];

    %Step 3: Expand the tree for a new node    
    x_new=[];
    theta = atan2d((x_rand(2)-x_near(2)),(x_rand(1)-x_near(1)));
    x_new = [(x_near(1) + Delta*cosd(theta)),(x_near(2) + Delta*sind(theta))];
    
        % check if the node is collision free
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    count=count+1;
    
    %Step 4: Add new node into the tree 'T'
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = x_near(1);
    T.v(count).yPrev = x_near(2);
    T.v(count).dist = Delta;
    T.v(count).indPrev = min_index_temp;    %   the index of parent node
    
    
    %Step 5: Check if it arrived in the vicinity of target
    dist_goal = sqrt((x_new(1)-x_Goal)^2 + (x_new(2)-y_Goal)^2);
    if (dist_goal<Thr)
        bFind = true;
        break;
    end
    
    %Step 6: Draw this process
    plot([x_near(1), x_new(1)],[x_near(2), x_new(2)],'b');
    plot(x_new(1), x_new(2), 'ko', 'MarkerSize',4, 'MarkerFaceColor','k');
   
    pause(0.05); % so that we can see it
end
%% Path found, acqiring solution
if bFind
    path.pos(1).x = x_Goal; path.pos(1).y = y_Goal;
    path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
    pathIndex = T.v(end).indPrev; % add target point to path
    j=0;
    while 1
        path.pos(j+3).x = T.v(pathIndex).x;
        path.pos(j+3).y = T.v(pathIndex).y;
        pathIndex = T.v(pathIndex).indPrev;
        if pathIndex == 1
            break
        end
        j=j+1;
    end  % 
    path.pos(end+1).x = x_Start; path.pos(end).y = y_Start; % Add start into the path
    for j = 2:length(path.pos)
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
end
