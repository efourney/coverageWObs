%% ShortesNodePairtPath: given two nodes (input as Node1, NOde2) which are not
%  obstacle vertices, and the list of obstacles (which is a global variable), 
%  this function determines the shortest path between the two nodes.  The function 
%  first checks if the shortest path is a straight line (there are no interferring
%  obstacles).  The function implicitly assumes that that nodes are not inside
%  an obstacle, or on the boundary of an obstacle. If a straight path connection
%  does not exist, then the Tangent graph (TG) is constructed, and the shortest 
%  path between the nodes is found from the TG.  The actual path is returned as a 
%  cell array, path_polyline, which contains the x--coordinates and y-coordinates of
%  the path.  The returned variable "weight" is the path length.
%  
function [path_polyline, weight] = ShortestNodePairPath(Node1,Node2)
global ObstacleBoundary_List
global nObst
%
%  first check if there is an unobstructed path between 
edge_x = [Node1(1),Node2(1)];  edge_y = [Node1(2),Node2(2)];
collides = false;
for i = 1:nObst
    % fprintf('  ShortestNodePairPath: processing obstacle %i\n',i);
    % [bx,by] = boundary(Obstacle_List(i));
    bx = ObstacleBoundary_List{i}{1};  by = ObstacleBoundary_List{i}{2};
    [x_int,y_int] = polyxpoly(edge_x,edge_y,bx,by,'unique');
    if ~isempty(x_int) 
        collides = true;
        % fprintf('ShortestNodePairPath: found obstacle collision between nodes\n');
        break;
    end
end
    % if there is no straight path, then construct the visibility graph and
    %  find the shortest path
if (collides == true)
    % fprintf('   Computing Node Pair Visibility Graph\n');
    NodesTangentGraph = NodePairTangentGraph(Node1,Node2);
    nTGnodes = numnodes(NodesTangentGraph);                    % get number of TVG nodes
    NodeLocations = NodesTangentGraph.Nodes.Locations;         % strip of the node locations
    [PathNodes,weight] = shortestpath(NodesTangentGraph,(nTGnodes-1),nTGnodes);
    nPath = length(PathNodes);                         % number of shortest path nodes
    % fprintf('     number of PathNodes in shortest path = %i\n',nPath);
    path_x = []; path_y = [];                          % initialize empty shortest path poly-line
    for j=1:nPath
        CurShortNode = NodeLocations{PathNodes(j)};
        path_x = [path_x, CurShortNode(1)];  path_y = [path_y, CurShortNode(2)];
        path_polyline = {path_x,path_y};
    end
else         % straight path between node1 and node 2 doesn't collide with obstacles
    % fprintf('ShortestNodePairPath:  no obstacle between nodes\n');
    path_x = [Node1(1),Node2(1)];    path_y = [Node1(2),Node2(2)];
    weight = norm(Node2-Node1);
    path_polyline = {path_x,path_y};
end  %  of if/else
end   % of function

