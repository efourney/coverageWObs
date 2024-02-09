%% ShortestTargetPath: give two Target nodes (input as Target1, Target2), and the 
%  list of obstacles (which is a global variable), this function determines
%  the shortest path between the two targets.  The function first checks if
%  the shortest path is a straight line (there are no interferring obstacles).  
%  If a straight path connections does not exist, then the tangent graph (TG) 
%  is constructed, and the shortest path is found from the TG.
%  
function [path_polyline,weight] = ShortestTargetPath(Target1,Target2)
global ObstacleBoundary_List
global nObst
%
%  first check if there is an unobstructed path between 
edge_x = [Target1(1),Target2(1)];  edge_y = [Target1(2),Target2(2)];    % build an edge between the two targets
collides = false;                                                       % initially, there is no interferring obstacle
for i = 1:nObst                                                         % check each obstacle for collision with the edge
    fprintf('  ShortestTargetPath: processing obstacle %i\n',i);
    bx = ObstacleBoundary_List{i}{1};   by = ObstacleBoundary_List{i}{2};
    [x_int,y_int] = polyxpoly(edge_x,edge_y,bx,by,'unique');            % look for collision using MATLIB polyxpoly
    if ~isempty(x_int)
        collides = true;
        fprintf('      In ShortestTargetPath: found collision between path to targets and obstacle %i\n',i);
        break;
    end
end
    % if there is no straight path, then construct the tangent graph and
    %  find the shortest path
if (collides==true)
    fprintf('      Computing Target Tangent Graph\n');
    TargetTangentGraph = NodePairTangentGraph(Target1,Target2);
    nTTGnodes = numnodes(TargetTangentGraph);            % get number of TTG nodes
    NodeLocations = TargetTangentGraph.Nodes.Locations;  % strip of the node locations
    [PathNodes,weight] = shortestpath(TargetTangentGraph,(nTTGnodes-1),nTTGnodes);
    %display(PathNodes);
    nPath = length(PathNodes);                           % number of shortest path nodes
    fprintf('     number of PathNodes in shortest path = %i\n',nPath);
    path_x = []; path_y = [];                          % initialize empty shortest path poly-line
    for j=1:nPath
        CurShortNode = NodeLocations{PathNodes(j)};
        path_x = [path_x, CurShortNode(1)];  path_y = [path_y, CurShortNode(2)];
        path_polyline = {path_x,path_y};
    end
else
    path_x = [Target1(1),Target2(1)];
    path_y = [Target1(2),Target2(2)];
    weight = norm(Target2-Target1);
    path_polyline = {path_x,path_y};
end  %  of if/else
end   % of function

