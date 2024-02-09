%% This function computes the portion of the Tangent Graph that arises from
%  the obstacles, and which does not change with the choice of start and goal
function [OVG] = ObstacleTangentGraph()
global Obstacle_List nObst
global ObstacleBoundary_List
global nObstacleVertices
global ObstacleVertexBase
%
ObstVertices = {};                           % cell array that stores the vertex coordinates
ObstIndices = {};                            % storage for obstacle indices associated to vertices
s = []; t=[]; w=[];                          % allocate space for the Obstacle visibility graph

%% Add the bounding edges of each obstacle.  The vertex indices and data 
%  established in this step are needed in subsequent steps
for i=1:nObst
    % fprintf('Processing Obstacle %i\n',i);
    bx = ObstacleBoundary_List{i}{1};        % x and y coordinates of obstacle vertices
    by = ObstacleBoundary_List{i}{2};
    nVert_i = nObstacleVertices(i);
        % loop through all of the vertices/edges of ith obstacle, and add edges to graph
    for j=1:(nVert_i-1)
        sNodeIndex = ObstacleVertexBase(i)+j;                     % integer index of jth node in obstacle i        
        tNodeIndex = sNodeIndex + 1;                              % next vertex along the boundary
        ObstVertices{sNodeIndex} = [bx(j),by(j)];                 % location of vertex, to store in graph
        ObstIndices{sNodeIndex} = i;                              % index of obstacle i that contains this vertex
        s = [s,sNodeIndex];  t = [t,tNodeIndex];
        wst = norm([bx(j+1),by(j+1)] - [bx(j),by(j)]);  % edge weight
        w = [w,wst];
        % fprintf('   j= %i, sNodeIndex = %i, tNodeIndex = %i, weight = %f\n',j,sNodeIndex,tNodeIndex,wst);
    end
        % make edge from last vertices to first vertex
    sNodeIndex = ObstacleVertexBase(i) + nVert_i;  tNodeIndex = ObstacleVertexBase(i) + 1;
    ObstIndices{sNodeIndex} = i;
    ObstVertices{sNodeIndex} = [bx(nVert_i),by(nVert_i)];         % store obstacle vertex
    s = [s,sNodeIndex]; t = [t,tNodeIndex];
    wst = norm([bx(1),by(1)]-[bx(nVert_i),by(nVert_i)]);
    w = [w,wst];
    % fprintf('last link:   j= %i, sNodeIndex = %i, tNodeIndex = %i, weight = %f\n',nVert_i,sNodeIndex,tNodeIndex,wst);
end  % of for i=1:nObst
%% Add the separating and supporting tangents between obstacle pairs. 
BiTangents = zeros(4,2);                     % allocate space for BiTangent Indices
for i=1:(nObst-1)
    bx_i = ObstacleBoundary_List{i}{1};   by_i = ObstacleBoundary_List{i}{2};
    for j=(i+1):nObst
        bx_j = ObstacleBoundary_List{j}{1};   by_j = ObstacleBoundary_List{j}{2};
        BiTangents = ObstacleBiTangents(i,j);  % construct the four bitangents
        for k=1:4                              % integrate bitangents into the graph
                % calculate the correct Tangent Graph Node index of each
                % end of the tangent edge
            sVertexIndex = BiTangents(k,1);  tVertexIndex = BiTangents(k,2);
            sgraph = ObstacleVertexBase(i) + sVertexIndex;
            tgraph = ObstacleVertexBase(j) + tVertexIndex;
            
            sLocation = [bx_i(sVertexIndex),by_i(sVertexIndex)];
            tLocation = [bx_j(tVertexIndex),by_j(tVertexIndex)];
            wt = norm(tLocation-sLocation);
            s = [s,sgraph];  t=[t,tgraph]; w = [w,wt];
        end   % of for k=1:4
    end       % of for j=i:nObst
end           % of for i=1:(nObst-1)
% fprintf('lengths: |s|=%i, |t|=%i, |w|=%i\n',length(s),length(t),length(w));
OVG = graph(s,t,w);                        % make the graph
OVG.Nodes.Locations = transpose(ObstVertices); % store the vertex coordinates
OVG.Nodes.Indices = transpose(ObstIndices);

%plot OVG
Locations = {};
%% first plot the obstacles
nObst = length(Obstacle_List);
for iplot = 1:nObst
    plot(Obstacle_List(iplot));
    hold on;
end
%% now plot the target nodes and sensing region constraints
    % First extract the target nodes from MaxGraph
Locations = OVG.Nodes.Locations;      % locations of Target Nodes

nTargets = size(Locations);
TargetNodes = [];
for i=1:nTargets                          % sloppy code to convert Cell array to conventional array
    %CurNode = Locations{i};
    TargetNodes(i,:) = Locations{i};      %
end

for i = 1:height(OVG.Edges)
    loc1 = TargetNodes(OVG.Edges.EndNodes(i,1),:);
    loc2 = TargetNodes(OVG.Edges.EndNodes(i,2),:);
    plot([loc1(1), loc2(1)],[loc1(2), loc2(2)])
end

end % of function
