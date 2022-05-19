# RRT_star_path_finding
This repo is for path finding.
RRT and its results. In RRT.m, the function of auto setting was added.

To use it, enter the environment settings in the 'Initialise process'.
Note that the origin of the map is at "Left Top" corner.

Set 'Auto_setting' as 'true' to enable setting up the 'Thr'(threshold for reaching vicinity of target),
'Delta' (The step size for growing the tree),
'iteration_num' (iteration number),
automatically based on the vertical size of the map image. 