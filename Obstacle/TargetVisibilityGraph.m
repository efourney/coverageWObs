%% Construct the Visibility Graph for two targets and the obstacles.  This
%  function assumes that the 'ObstacleVisibilityGraph' has already been
%  computed.  This version is a "brute force" version which is inefficient
%
function [TVG] = TargetVisibilityGraph(Target1,Target2)
global Obstacle_List
global Obstacle_VG                     % visibility graph of obstacles, without target start/goal nodes
%
TVG = Obstacle_VG;                      % make a copy of the obstacle VG to be modified
nObst = length(Obstacle_List);
IndexToBase = 0;
nObstVertices = numnodes(Obstacle_VG)    % number of nodes already in Obstacle Graph
s1 = nObstVertices + 1;                   % index of node representing Target 1 in the TVG
s2 = nObstVertices + 2;                   % index of node representing Target 2 in the TVG
    % loop through each obstacle and its vertices.  See if there is a free
    % edge connecting Target 1 and Target 2
for i=1:nObst
    [bToX,bToY] = boundary(Obstacle_List(i));
    nVerticesTo = length(bToX) -1;
    % fprintf('TargetVisibilityGraph: Processing Obstacle %i\n',i);
    for j=1:nVerticesTo
        % see if the vertex is visible from Target 1
        VertexTo = [bToX(j),bToY(j)];
        VToIndex = IndexToBase + j;
        % fprintf('  processing Vertex %i\n',j);
        edgePoly1_x = [Target1(1),VertexTo(1)];  edgePoly2_x = [Target2(1),VertexTo(1)];  
        edgePoly1_y = [Target1(2),VertexTo(2)];  edgePoly2_y = [Target2(2),VertexTo(2)];
        [x_int1,y_int1] = polyxpoly(edgePoly1_x,edgePoly1_y,bToX,bToY,'unique');
        [x_int2,y_int2] = polyxpoly(edgePoly2_x,edgePoly2_y,bToX,bToY,'unique');
        int1_length = length(x_int1);        int2_length = length(x_int2);
            % if only 1 intersection with To Obstacle, then vertex is visible
        if int1_length==1
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
                % fprintf('    found a free edge from Target 1 to Obstacle %i, Vertex %i\n',i,j);
                ws1v = norm(Target1-VertexTo);
                TVG = addedge(TVG,s1,VToIndex,ws1v);
            end
        end      %  if int_length
        if int2_length==1
            collides = false;
            for m=1:nObst
                if (m ~= i)
                    [bmx,bmy]=boundary(Obstacle_List(m));
                    [x_alt2,y_alt2] = polyxpoly(edgePoly2_x,edgePoly2_y,bmx,bmy,'unique');
                    if ~isempty(x_alt2)
                        collides = true;
                        % fprintf('      found interior collision for Target 2\n');
                        break
                    end  % if ~isempty
                end      % if (m ~= i)
            end          % for m
            if (collides == false)
                % fprintf('    found a free edge from Target 2 to Obstacle %i, Vertex %i\n',i,j);
                ws2v = norm(Target2-VertexTo);
                TVG = addedge(TVG,s2,VToIndex,ws2v);
            end
        end      %  if int_length
    end          % for j
    IndexToBase = IndexToBase + nVerticesTo;               % update base for "TO" vertices
end % of for i
TVG.Nodes.Locations{nObstVertices+1} = Target1; TVG.Nodes.Locations{nObstVertices+2} = Target2;
TVG.Nodes.Indices{nObstVertices+1} = 'T1'; TVG.Nodes.Indices{nObstVertices+2} = 'T2';

end %function
