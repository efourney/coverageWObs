%% Construct the Visibility Graph for two nodes and the obstacles.  This
%  function assumes that the 'ObstacleVisibilityGraph' has already been
%  computed.  This algorithm also assumes that the nodes doe not lie in 
%  any obstacle.  This version is a "brute force" version which is inefficient
%
function [PVG] = NodePairVisibilityGraph(Node1,Node2)
global Obstacle_List nObst
global Obstacle_VG                        % visibility graph of obstacles, without node pair
%
PVG = Obstacle_VG;                        % make a copy of the obstacle VG to be modified
IndexToBase = 0;                          % a counter to properly index obstacle vertices.
nObstVertices = numnodes(Obstacle_VG);    % number of nodes already in Obstacle Graph
s1 = nObstVertices + 1;                   % index of node1 (1 higher than nObstVertices) 
s2 = nObstVertices + 2;                   % index of node2 
    % loop through each obstacle and its vertices.  See if there is a free
    % edge connecting NOde 1 and Node 2
for i=1:nObst                             % loop through each obstacle
    [bToX,bToY] = boundary(Obstacle_List(i));          % get polyline of obstacle boundary
    nVerticesTo = length(bToX) -1;                     % no. of vertices on ith obstacle
    % fprintf('NodePairVisibilityGraph: Processing Obstacle %i\n',i);
    for j=1:nVerticesTo                                % loop through each vertex in ith obstacle
        % see if the vertex is visible from Node1
        VertexTo = [bToX(j),bToY(j)];                  % jth vertex on ith obstacle boundary
        VToIndex = IndexToBase + j;                    % index of this vertex
        % fprintf('  processing Vertex %i\n',j);
            % prepare polylines that are used for intersection test. Then check 
            % the intersection of Node1-vertex edge with ith Obstacle.  If only one intersection
            % (which will be the vertex on the obstacle boundary), then the
            % edge might be added to the visibility graph. If there are no intersections,
            % then there is some fault--but this code does not check for
            % that fault.  If there are 2 or more intersections, then the
            % putative edge cross the ith obstacle, and so that the edge is
            % not viable
        edgePoly1_x = [Node1(1),VertexTo(1)];  edgePoly2_x = [Node2(1),VertexTo(1)];  
        edgePoly1_y = [Node1(2),VertexTo(2)];  edgePoly2_y = [Node2(2),VertexTo(2)];
        [x_int1,y_int1] = polyxpoly(edgePoly1_x,edgePoly1_y,bToX,bToY,'unique');
        [x_int2,y_int2] = polyxpoly(edgePoly2_x,edgePoly2_y,bToX,bToY,'unique');
        int1_length = length(x_int1);        int2_length = length(x_int2);
            % If the putative edge does not intersect the ith obstacle,
            % then we must check if the edge crosses some other obstacle.
        if int1_length == 1
            collides = false;
            for m=1:nObst
                if (m ~= i)
                    [bmx,bmy]=boundary(Obstacle_List(m));
                    [x_alt1,y_alt1] = polyxpoly(edgePoly1_x,edgePoly1_y,bmx,bmy,'unique');
                    if ~isempty(x_alt1)
                        collides = true;
                        % fprintf('      found interior collision\n');
                        break
                    end  % if ~isempty
                end      % if (m ~= i)
            end          % for m
            if (collides == false)
                % fprintf('    found a free edge from Node1 to Obstacle %i, Vertex %i\n',i,j);
                ws1v = norm(Node1-VertexTo);
                PVG = addedge(PVG,s1,VToIndex,ws1v);
            end
        end      %  if int_length
            % now perform the same tests with Node 1 and the same ith obstacle
        if int2_length==1
            collides = false;
            for m=1:nObst
                if (m ~= i)
                    [bmx,bmy]=boundary(Obstacle_List(m));
                    [x_alt2,y_alt2] = polyxpoly(edgePoly2_x,edgePoly2_y,bmx,bmy,'unique');
                    if ~isempty(x_alt2)
                        collides = true;
                        % fprintf('      found interior collision for Node 2\n');
                        break
                    end  % if ~isempty
                end      % if (m ~= i)
            end          % for m
            if (collides == false)
                % fprintf('    found a free edge from Node2 to Obstacle %i, Vertex %i\n',i,j);
                ws2v = norm(Node2-VertexTo);
                PVG = addedge(PVG,s2,VToIndex,ws2v);
            end
        end      %  if int_length
    end          % for j
    IndexToBase = IndexToBase + nVerticesTo;               % update base for "TO" vertices
end      % of for i
PVG.Nodes.Locations{nObstVertices+1} = Node1; PVG.Nodes.Locations{nObstVertices+2} = Node2;
PVG.Nodes.Indices{nObstVertices+1} = 'T1'; PVG.Nodes.Indices{nObstVertices+2} = 'T2';
end      % of function
