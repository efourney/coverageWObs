%% This function computes the portion of the Visibility Graph that arises from
%  the obstacles, and which does not change with the choice of start and
%  goal
function [OVG] = ObstacleVisibilityGraph()
global Obstacle_List

nObst = length(Obstacle_List);
ObstVertices = {};                           % cell array that stores the vertex coordinates
Vcounter = 1;                                % keep track of # of vertices added
ObstIndex = {};                              % storage for obstacle indices associated to vertices
s = []; t=[]; w=[];                          % allocate space for the Obstacle visibility graph

%% Add the bounding edges of each obstacle to the Visibility Graph.  The vertex 
%  indices and data established in this step are needed in subsequent steps
for i=1:nObst
    [bx,by] = boundary(Obstacle_List(i));    % x and y coordinates of obstacle vertices
    nObstVertices = length(bx) - 1;          % number of vertices in Obstacle i
    ObstIndex{Vcounter}= i;                  % store the obstacle index
    ObstVertices{Vcounter} = [bx(1),by(1)];  % first vertex coordinate
    Vtemp = Vcounter;                        % store index of first obstacle verte
    for j=1:(nObstVertices-1)
        ObstVertices{Vcounter+1} = [bx(j+1),by(j+1)];
        s = [s,Vcounter]; t = [t,Vcounter + 1];       % edge from Vcounter node to Vcounter + 1 node
        wst = norm(ObstVertices{Vcounter+1} - ObstVertices{Vcounter});  % edge weight
        w = [w,wst];
        ObstIndex{Vcounter} = i;             % store associated obstacle index
        Vcounter = Vcounter + 1;             % increase the vertex counter
    end
    s = [s,Vcounter]; t = [t,Vtemp];         % make edge from last vertex to 1st vertex
    wst = norm([bx(nObstVertices),by(nObstVertices)]-[bx(1),by(1)]);
    w = [w,wst];
    ObstIndex{Vcounter} = i;
    Vcounter = Vcounter + 1;    
end
%% Add the edges between the vertices of different obstacles.  This is currently 
%  done in a brute-force way that leads to O(n^3) complexity.  It would be 
%  better to use Lee's algorithm to figure out vertex-vertex visibility in
%  O(n^2 log(n))
IndexFromBase = 0;                                   % help count vertex index of "from" obstacle
IndexToBase = 0;                                     % help count vertex index of "to" obstacle
for i = 1:(nObst-1)
    [bFromX,bFromY] = boundary(Obstacle_List(i));    % boundary of "From" object
    nVerticesFrom = length(bFromX) - 1;              % no. of distinct vertices
    if i==1
        IndexToBase = nVerticesFrom;                 % set base for "TO" vertices
    end
    for j=(i+1):nObst
        [bToX,bToY] = boundary(Obstacle_List(j));
        nVerticesTo = length(bToX)-1;
        for k=i:nVerticesFrom
            VertexFrom = [bFromX(k),bFromY(k)];
            VFromIndex = IndexFromBase + k;
            for m=1:nVerticesTo
                VertexTo = [bToX(m),bToY(m)];
                VToIndex = IndexToBase + m;
                edgePoly_x = [VertexFrom(1),VertexTo(1)];  
                edgePoly_y = [VertexFrom(2),VertexTo(2)];
                [x_int,y_int] = polyxpoly(edgePoly_x,edgePoly_y,bToX,bToY,'unique');
                int_length = length(x_int);
                    % if only 1 intersection with To Obstacle, then check From Obstacle
                if int_length==1
                       % check if edge crosses the interior of the "from" obstacle
                    [x_int,y_int] = polyxpoly(edgePoly_x,edgePoly_y,bFromX,bFromY,'unique');
                    if length(x_int)==1                        % edge is obstacle-free
                        fprintf('found a free edge from Vertex %i to Vertex %i\n',VFromIndex,VToIndex);
                        s = [s,VFromIndex]; t = [t,VToIndex];  % add edge to OVG
                        wst = norm(VertexTo-VertexFrom);
                        w = [w,wst];                           % add edge weight
                    end  % if length()
                end      % if int_length
            end  % for m
        end      % for k
        IndexToBase = IndexToBase + nVerticesTo;               % update base for "TO" vertices
    end          % for j
    IndexFromBase = IndexFromBase + nVerticesFrom;
end              % for i
%fprintf('lengths: |s|=%i, |t|=%i, |w|=%i\n',length(s),length(t),length(w));
% display(s); display(t); display(w);
OVG = graph(s,t,w);                        % make the graph
OVG.Nodes.Locations = transpose(ObstVertices); % store the vertex coordinates
OVG.Nodes.Indices = transpose(ObstIndex);
end % of function
        