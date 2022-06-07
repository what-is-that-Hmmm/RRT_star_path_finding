function final_PATH = RRT_star_path_finding(x_Start,y_Start,x_Goal,y_Goal,...
    Thr,Delta,iteration_num,greedy_index,PNGmap)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Inputs: 
% Thr:              target threshold (vicinity to be considered as target)
% Delta:            step length
% iteration_num:    the iteration number for expanding the tree
% greedy_index:     how greedy the tree would expand towards target, 1 for 
%                   the most greedy.
% PNGmap:           A map in png format.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

query_range = 2*Delta; 
% Build up initial tree
T.v(1).x = x_Start;         % 'T' as tree, 'v' as node, adding starting point to the tree.
T.v(1).y = y_Start; 
T.v(1).xPrev = x_Start;     % initial parent node is it self.
T.v(1).yPrev = y_Start;
T.v(1).dist=0;          % Euclidean distance from parten not to current node.
T.v(1).indPrev = 0;   

% Setup the map
figure(1);

% Imp=rgb2gray(PNGmap);
Imp = PNGmap;
imshow(Imp)
xL=size(Imp,2);     %length of map in X
yL=size(Imp,1);     %length of map in Y
hold on

% plot starting point and goal point.
plot(x_Start, y_Start, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
plot(x_Goal, y_Goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');% plot start and target point

% Tree growing
path.cost = (xL^2+yL^2)^2;  %   cost of a path, make it large so that it can be updated
path.pos.x = 0;
path.pos.y = 0;
count=1;        % indicating the position in tree 'T'
parent_point = [0,0];
bFind = false;  % if there is a path? No path yet
aFind = false;

for iter = 1:iteration_num
    %% finding X_new

    
    % Randomly sample on the map
    if (rand > greedy_index)     % randomly sample in the map, greedy-epsilon
        x_temp = floor(rand*xL);
        y_temp = floor(rand*yL);
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
    
    % Shaping the tree, connect X_new to the tree     
    Xnear = [];
    %**********************************************************************
    %Xnear format:
    % |1,x pos|2,y pos|3,previous x pos|4,previous y pos|5,DP
    % dist with X_new|6,position in tree|7,acc dist|
    %**********************************************************************
    
    Xnear_index = 1;
    %   This loop is for finding the most suitable point on the node within
    %   query range. 
    for i2=1:size(T.v,2)
       NearpointDistance = sqrt((x_new(1)-T.v(i2).x)^2 + (x_new(2)-T.v(i2).y)^2);
       if (NearpointDistance < query_range)
           if ~collisionChecking([T.v(i2).x,T.v(i2).y], x_new,Imp)
               continue
           end
           % record the points within query range
           Xnear(Xnear_index, :) = [T.v(i2).x, T.v(i2).y, T.v(i2).xPrev,...
               T.v(i2).yPrev, (T.v(i2).dist+NearpointDistance), i2, T.v(i2).dist];
           Xnear_index = Xnear_index + 1;
       end     
    end     % end of the for loop for findin the points within query range
    
    
    if ~isempty(Xnear)
        Xnear = sortrows(Xnear, 5, 'descend');
        x_min = Xnear(end, [1,2]);
        parent_point = x_min; 

        %  Add new node into the tree 'T'
        T.v(count).x = x_new(1);
        T.v(count).y = x_new(2);
        T.v(count).xPrev = parent_point(1);
        T.v(count).yPrev = parent_point(2);
        T.v(count).dist = Xnear(end, 5);     % accumulative distance
        T.v(count).indPrev = Xnear(end, 6);    %   the index of parent node

        Xnear(end+1,:) = [T.v(count).x,T.v(count).y,T.v(count).xPrev,...
            T.v(count).yPrev,T.v(count).dist,i2+1,T.v(count).dist];
        Xnear = sortrows(Xnear, 7, 'descend');
        Xnew_temp_index = find(Xnear(:,6)==i2+1);
        temp_ind = Xnew_temp_index(end);
        
        if (temp_ind ~= 1)
            for i4=1:temp_ind
                % all points in 'Xnear' list are feasible to 'x_new'
                % calculate the distance to the new point
                ToXnew_temp = sqrt((Xnear(i4,1)-x_new(1))^2 + (Xnear(i4,2)-x_new(2))^2);
                DP_temp = T.v(count).dist + ToXnew_temp;
                if (DP_temp < Xnear(i4,7))  % check if go through x_new is shorter
                    %   rewire to x_new
                    T.v(Xnear(i4,6)).xPrev = x_new(1);
                    T.v(Xnear(i4,6)).yPrev = x_new(2);
                    T.v(Xnear(i4,6)).dist = DP_temp;
                    T.v(Xnear(i4,6)).indPrev = i2+1;
                end
            end     %end of the for loop of rewiring
        end    
    end
    
    %% ready for next cycle
    count=count+1;   
    
    % Check if it arrived in the vicinity of target
    dist_goal = sqrt((x_new(1)-x_Goal)^2 + (x_new(2)-y_Goal)^2);
    
    if (dist_goal<Thr)
        bFind = true;
        
        % store previous path & cost for further comparison
        pre_path = path.pos;
        pre_cost = path.cost;
        
        % add target point to path
        path.pos(1).x = x_Goal; 
        path.pos(1).y = y_Goal;
        
        path.pos(2).x = T.v(end).x; 
        path.pos(2).y = T.v(end).y;
        
        pathIndex = T.v(end).indPrev; % referring to the last second point
        j2=3;
        while 1
            if isempty(T.v(pathIndex).x)
                warning('q(-_-)p : Data corrupted, environment restarting...... ');
                clear T;
                clear path;
                bFind = false;
                % restart the tree
                T.v(1).x = x_Start;         % 'T' as tree, 'v' as node, adding starting point to the tree.
                T.v(1).y = y_Start;
                T.v(1).xPrev = x_Start;     % initial parent node is it self.
                T.v(1).yPrev = y_Start;
                T.v(1).dist=0;          % Euclidean distance from parten not to current node.
                T.v(1).indPrev = 0;
                
                % restart the map
                figure(1);
                hold off
                imshow(Imp)
                xL=size(Imp,2);     %length of map in X
                yL=size(Imp,1);     %length of map in Y
                hold on
                
                % plot starting point and goal point.
                plot(x_Start, y_Start, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
                plot(x_Goal, y_Goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');
                disp('\(^_^)/ : Environment restarted!');
                path.cost = (xL+yL)*100;  %   cost of a path
                path.pos.x = 0;
                path.pos.y = 0;
                count=1;        % indicating the position in tree 'T'
                parent_point = [0,0];
                break
            end
            
            %   add nodes from 'T' to path 
            path.pos(j2).x = T.v(pathIndex).x;
            path.pos(j2).y = T.v(pathIndex).y;
            pathIndex = T.v(pathIndex).indPrev;
            if pathIndex == 1
                break
            end
            j2=j2+1;
        end  % 
        
        if bFind    % present the path found
            % Add start into the path
            path.pos(end+1).x = x_Start; 
            path.pos(end).y = y_Start; 
            path.cost = 0;
            for j2 = 2:length(path.pos)
                % display the path
                path.cost = (path.pos(j2).x - path.pos(j2-1).x)^2 + (path.pos(j2).y...
                    - path.pos(j2-1).y)^2 + path.cost;
                plot([path.pos(j2).x; path.pos(j2-1).x;], [path.pos(j2).y; path.pos(j2-1).y],...
                    'y', 'Linewidth', 2);
                
            end
            
            % store previous path & cost for further comparison
            if (~aFind)     % calibrating path cost
                pre_path = path.pos;
                pre_cost = path.cost;
                disp('Path cost calibrated');
            end
            aFind = true;

            disp('One path found');
        end
        bFind = false;
        if (pre_cost < path.cost)  %||((aFind && ~bFind))
            path.pos = pre_path;
            path.cost = pre_cost;
            disp('Maintain original path');
            % plot the current optimal path
            path.cost = 0;
            for j2 = 2:length(path.pos)
                % display the path
                path.cost = (path.pos(j2).x - path.pos(j2-1).x)^2 + (path.pos(j2).y...
                    - path.pos(j2-1).y)^2 + path.cost;
                plot([path.pos(j2).x; path.pos(j2-1).x;], [path.pos(j2).y; path.pos(j2-1).y],...
                    'r', 'Linewidth', 2);
            end 
            
        else
            
            path.cost = 0;
            for j2 = 2:length(path.pos)
                % display the path
                path.cost = (path.pos(j2).x - path.pos(j2-1).x)^2 + (path.pos(j2).y...
                    - path.pos(j2-1).y)^2 + path.cost;
                plot([path.pos(j2).x; path.pos(j2-1).x;], [path.pos(j2).y; path.pos(j2-1).y],...
                    'r', 'Linewidth', 2);
            end
            disp('Path updated');
        end    
    end    
end   % end of interation of 'for' loop for growing tree
%% Path found, acqiring solution
if aFind
    disp('Sucess, path found.');
    for j2 = 2:length(path.pos)
        plot([path.pos(j2).x; path.pos(j2-1).x;], [path.pos(j2).y; path.pos(j2-1).y], 'c', 'Linewidth', 3);
    end
    title("RRT* findings");
else
    error('(||-_-)~Error : RRT* searching failed. No path found.');
end

final_PATH = path;
end

