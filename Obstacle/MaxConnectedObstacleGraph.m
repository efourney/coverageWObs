%% function to make a maximally connected graph from the target nodes while 
%  including the effects of obstacles.  Note, the start node and goal node are 
%  not connected in this graph, as we do not want to allow this path.  
%
%  The output of this function is a maximally connected graph of the target 
%  nodes.  The edges of the graph reflect the ability of the robot to move 
%  between the nodes that are connected by the edge.  The Target Node 
%  locations are added as attributes to the Target Nodes.  Likewise,
%  information about the shortest path between nodes is added as an
%  attribute called "ShortPath".  Note that if short path is empty, then it
%  is assumed that there are no obstacles between vertices, and the path
%  consists of a straight line connecting the Target Nodes.  If the shortest
%  path is not a straight line (because there are intervening obstacles),
%  then "ShortPath" contains a list of nodes that define the shortest path
%  (but not including the two target vertices at either end of the path).
%
%  INPUTS: InNodes are the target nodes: we assume that the last Target Node 
%          in the list is the goal node, and the first is the Start Node.
%
%  DATA STRUCTURE NOTES: In the maximally connected graph, an edge between 
%  two target nodes only indicates that there exists a path between the two 
%  targets.  The actual path is another annoted variable, "Path".  If an edge 
%  exists, then its associated path will either be a straight line if there 
%  are no impeding obstacles, or a shortest path. The "Path" variable
%  contains the x- and y-coordinates of the shortest path vertices.
%
function [MaxGraph] = MaxConnectedObstacleGraph(InNodes)
%
FirstNode = InNodes(1,:);               % extract the starting node 
n_nodes = length(InNodes);              % number of nodes
LastNode  = InNodes(n_nodes,:);         % goal node
NodeLocations = {};                     % initialize cell array for node locations
Paths = {};                             % initialize cell array for Shortest Paths
path_iter = 1;                          % iteration to keep track of paths
%% Now build the graph
s = []; t = []; w = [];                 % allocated space for edges/nodes and weights
    % build edges between StartNode and all nodes, except Goal Node.
for i=2:(n_nodes-1)                     % make connections between start nodes and all nodes except goal node t
    fprintf('MaxGraph: connecting Start Node to Target(%i)\n',i);
    % determine the shortest path between s1 and each subsequent target
    [edge_path,wsi] = ShortestTargetPath(FirstNode,InNodes(i,:));
    % if a connecting path exists (edge_path will not be null), add the nodes 
    % to the graph, the weight w, and then the information about the path.
    s = [s,1]; t = [t,i];  w = [w,wsi];        % add edge and its weight (travel distance)
    Paths{path_iter} = edge_path;                  % store (possibly empty) edge path in Paths  
    path_iter = path_iter +1;
end
    % Now build max graph edges between nodes N_2,...,N_n
for i=2:(n_nodes-1)
    for j=(i+1):n_nodes
        fprintf('MaxGraph: connecting target node %i to target node %i\n',i,j);
        Node_i = InNodes(i,:);   Node_j = InNodes(j,:);
        % check if path from Node 1 to Node j exists, and its properties 
        [edge_path,wij] = ShortestTargetPath(Node_i,Node_j);
        s = [s, i];    t = [t, j]; w = [w, wij];
        Paths{path_iter} = edge_path;
        path_iter = path_iter + 1;
        fprintf('path_iter = %i\n',path_iter)
    end   % for j
end       % for i
for j=1:n_nodes                              % annotate graph with node locations
    NodeLocations{j} = InNodes(j,:);
end
% fprintf('  Length of NodeLocations is %i\n',length(NodeLocations));
% display(NodeLocations);
MaxGraph = graph(s,t,w);
nMGnodes = numnodes(MaxGraph); nMGedges = numedges(MaxGraph);
nPaths = length(Paths);
fprintf('MaxGraph: num nodes = %i; num edges = %i; no of Paths = %i\n',nMGnodes,nMGedges,nPaths);
MaxGraph.Nodes.Location = transpose(NodeLocations);
MaxGraph.Edges.Paths = transpose(Paths);
end

