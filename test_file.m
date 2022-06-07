clc
clear all; close all;

x_Start=1; y_Start=1;           % set starting point
x_Goal=2000; y_Goal=1400;       % set target point
Thr=150;                 % target threshold (vicinity to be considered as target)
Delta= 100;              % step length
iteration_num = 3000;   % the iteration number for expanding the tree
greedy_index = 0.01;     % how greedy the tree would expand towards target, 1 for the most greedy

ImpRgb=imread('usr_map1.png');

%% RRT_star
path = RRT_star_path_finding(x_Start,y_Start,x_Goal,y_Goal,...
    Thr,Delta,iteration_num,greedy_index,ImpRgb);

%% ploting
figure(2),
imshow(ImpRgb);
hold on
% plot starting point and goal point.
plot(x_Start, y_Start, 'ro', 'MarkerSize',5, 'MarkerFaceColor','r');
plot(x_Goal, y_Goal, 'go', 'MarkerSize',5, 'MarkerFaceColor','g');
for j2 = 2:length(path.pos)
    plot([path.pos(j2).x; path.pos(j2-1).x;], [path.pos(j2).y; path.pos(j2-1).y], 'c', 'Linewidth', 3);
    pause(0.01);
end
title('Path shown as light blue line');
hold off
disp("Finally path found");