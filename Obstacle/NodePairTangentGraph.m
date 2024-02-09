%% Construct the Tangent Graph for the two input nodes (Node1 and Node 2) and an obstacle.  
%  This function assumes that the 'ObstacleTangentGraph' (Obstacle_TG) has already been
%  computed.  This algorithm also assumes that the nodes do not lie in any obstacle.  
%
function [PTG] = NodePairTangentGraph(Node1,Node2)
global Obstacle_List nObst
global nObstacleVertices ObstacleVertex_List
global ObstacleBoundary_List
global Obstacle_TG                             % tangent graph of obstacles, without node pair
%
PTG = Obstacle_TG;                             % make a copy of the obstacle VG to be modified
IndexToBase = 0;                               % a counter to properly index obstacle vertices.
nObstGraphVertices = numnodes(Obstacle_TG);    % number of nodes already in Obstacle Graph
    % need to assign indices to new nodes.  They will be 1 and 2 more than
    % number of current nodes in graph.
s1 = nObstGraphVertices + 1;                        % index of node 1 (1 higher than nObstVertices) 
s2 = nObstGraphVertices + 2;                        % index of node 2 
    % The edges of the tangent graph have already been built for the obstacles.  Now
    % we must add possible edges from the nodes.  loop through each obstacle and its 
    % vertices to see if there is a free edge connecting NOde 1 and Node 2
    % to the given vertex on the ith obstacle
for i=1:nObst                             % loop through each obstacle
    bToX = ObstacleBoundary_List{i}{1};                  % get polyline of obstacle boundary
    bToY = ObstacleBoundary_List{i}{2};
    nVerticesTo = nObstacleVertices(i);                        % no. of vertices on ith obstacle
    % fprintf('NodePairVisibilityGraph: Processing Obstacle %i\n',i);
    for j=1:nVerticesTo                                % loop through each vertex in ith obstacle
            % see if the jth vertex of Obstacle i is visible from Node 1
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
                % loop through all other obstacles besides obstacle i
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
                PTG = addedge(PTG,s1,VToIndex,ws1v);
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
                PTG = addedge(PTG,s2,VToIndex,ws2v);
            end
        end      %  if int_length
    end          % for j
    IndexToBase = IndexToBase + nVerticesTo;               % update base for "TO" vertices
end      % of for i
PTG.Nodes.Locations{nObstGraphVertices+1} = Node1; 
PTG.Nodes.Locations{nObstGraphVertices+2} = Node2;
PTG.Nodes.Indices{nObstGraphVertices+1} = 'T1'; 
PTG.Nodes.Indices{nObstGraphVertices+2} = 'T2';
end      % of function
