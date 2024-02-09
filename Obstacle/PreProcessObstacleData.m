%% preprocess the problem data to look for problems, and to make sure that 
%  the data is in the assumed format.
function PreProcessObstacleData()
global Obstacle_List nObst
global ObstacleBoundary_List
global ObstacleVertex_List
global Obstacle_TG
    % Some of the tangent graph constructions assume that the polygon obstacle 
    % boundaries are listed in counter-clockwise order.  This step takes the 
    % given polygon obstacle inputs and ensures CCW ordering of boundaries.
for i=1:nObst
    [bx,by] = boundary(Obstacle_List(i));           % get ith obstacle boundary
    [bx_ccw,by_ccw] = poly2ccw(bx,by);              % put boundary points in CCW order
    ObstacleBoundary_List{i} = {bx_ccw,by_ccw};     % store obstacle i boundary in CCW format
    bx_ccw(end)=[]; by_ccw(end)=[];                 % get rid of repeat initial/end vertex
    ObstacleVertex_List{i} = {bx_ccw,by_ccw};       % store obstacle i vertices in CCW format
end % of for 
%Obstacle_TG = ObstacleTangentGraph() ;           % construct the obstacle visibility graph
end % of function PreProcess