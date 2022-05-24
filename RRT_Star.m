%% Initialise process
clc
clear all; close all;

x_Start=1; y_Start=1;           % set starting point
x_Goal=700; y_Goal=700;       % set target point
Thr=40;                 % target threshold (vicinity to be considered as target)
Delta= 30;              % step length
iteration_num = 3000;   % the iteration number for expanding the tree
greedy_index = 0;     % how greedy the tree would expand towards target, 1 for the most greedy
query_range = 2*Delta; 

%% Build up initial tree
T.v(1).x = x_Start;         % 'T' as tree, 'v' as node, adding starting point to the tree.
T.v(1).y = y_Start; 
T.v(1).xPrev = x_Start;     % initial parent node is it self.
T.v(1).yPrev = y_Start;
T.v(1).dist=0;          % Euclidean distance from parten not to current node.
T.v(1).indPrev = 0;   

%% Setup the map
figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
xL=size(Imp,2);     %length of map in X
yL=size(Imp,1);     %length of map in Y
hold on

% plot starting point and goal point.
plot(x_Start, y_Start, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
plot(x_Goal, y_Goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');% plot start and target point

%% Tree growing
temp_min_dist = xL;
path_cost = 0;  %   cost of a path
count=1;        % indicating the position in tree 'T'
parent_point = [0,0];
bFind = false;  % if there is a path 


for iter = 1:iteration_num
    % Randomly sample on the map
    x_rand=[];
    if (rand > greedy_index)     % randomly sample in the map, greedy-epsilon
        x_temp = floor(rand*800);
        y_temp = floor(rand*800);
        x_rand = [x_temp, y_temp];
    else
        x_rand = [x_Goal, y_Goal];
    end
        
    % find the nearest node in the tree to 'x_rand'
    min_distance = sqrt((x_rand(1)-T.v(1).x)^2 + (x_rand(2)-T.v(1).y)^2);
    min_index_temp = 1;
    if (iter >= 3)
        for i1=2:size(T.v,2)
            NearpointDistance = sqrt((x_rand(1)-T.v(i1).x)^2 + (x_rand(2)-T.v(i1).y)^2);
            if (NearpointDistance<min_distance)
                min_distance = NearpointDistance;
                min_index_temp = i1;
            end     % end of the if for comparing the smaller value
        end     % end of the for loop for findin the nearest node
    end     % end of the if for judging 'iter'
    x_near = [T.v(min_index_temp).x, T.v(min_index_temp).y];

    % Expand the tree for a new node    
    theta = atan2d((x_rand(2)-x_near(2)),(x_rand(1)-x_near(1)));
    x_new = [(x_near(1) + Delta*cosd(theta)),(x_near(2) + Delta*sind(theta))];
    
    if ~point_collision_check(x_new,Imp) 
       continue
    end
    
    %   check points in query range
    temp_min_dist = Inf; % make the distance large to update it
    
    %   This loop is for finding the most suitable point on the node within
    %   query range. 
    for i2=1:size(T.v,2)
     NearpointDistance = sqrt((x_new(1)-T.v(i2).x)^2 + (x_new(2)-T.v(i2).y)^2);
     
       if (NearpointDistance < query_range)
           x_temp = [T.v(i2).x, T.v(i2).y]; 
           
           % if the trace is feasible, then calculate DP distance
           % dp_distance:   DP distance is the accumulative distance to
           %                reach the new point 'x_new'.
           if collisionChecking(x_temp,x_new,Imp) 
              dp_distance = NearpointDistance + T.v(i2).dist;
              
              if ((dp_distance < temp_min_dist))
                  % If the accumulative distance is smaller, then connect
                  % the tree.
                  
                  %     erase previous connection 
%                   if collisionChecking(parent_point,x_new,Imp) 
%                        plot([parent_point(1), x_new(1)],[parent_point(2), x_new(2)],'w');
%                   end 
                  temp_min_dist = dp_distance;
                  parent_index = i2;
                  parent_point = x_temp;
                  
                  %  Add new node into the tree 'T'
                  T.v(count).x = x_new(1);
                  T.v(count).y = x_new(2);
                  T.v(count).xPrev = parent_point(1);
                  T.v(count).yPrev = parent_point(2);
                  T.v(count).dist = temp_min_dist + T.v(parent_index).dist;     % accumulative distance
                  T.v(count).indPrev = parent_index;    %   the index of parent node
%                   %   Draw the growing process
%                   plot([parent_point(1), x_new(1)],[parent_point(2), x_new(2)],'g');
%                   plot(x_new(1), x_new(2), 'bo', 'MarkerSize',4, 'MarkerFaceColor','b');
                  
                  %  Rewiring process 

                  pause(0.05); % so that we can see it                  
              end
           end
       end     
    end     % end of the for loop for findin the points within query range
    %   Draw the growing process
    plot([parent_point(1), x_new(1)],[parent_point(2), x_new(2)],'g');
    plot(x_new(1), x_new(2), 'bo', 'MarkerSize',4, 'MarkerFaceColor','b');
    
    count=count+1;   
    
    % Check if it arrived in the vicinity of target
    dist_goal = sqrt((x_new(1)-x_Goal)^2 + (x_new(2)-y_Goal)^2);
    
    if (dist_goal<Thr)
        bFind = true;
        
        % add target point to path
        path.pos(1).x = x_Goal; 
        path.pos(1).y = y_Goal;
        
        path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
        pathIndex = T.v(end).indPrev; % referring to the last second point
        j=3;
        while 1
            path.pos(j).x = T.v(pathIndex).x;
            path.pos(j).y = T.v(pathIndex).y;
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            j=j+1;
        end  % 
        
        % Add start into the path
        path.pos(end+1).x = x_Start; 
        path.pos(end).y = y_Start; 
        
        for j = 2:length(path.pos)
            path_cost = (path.pos(j).x - path.pos(j-1).x)^2 + (path.pos(j).y...
                - path.pos(j-1).y)^2 + path_cost;
            plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'y', 'Linewidth', 2);
        end
    end
end     % end of interation for growing tree


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
    Error('Error, no path found!');
end