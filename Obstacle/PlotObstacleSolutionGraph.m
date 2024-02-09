%% Plot the locations of (potentially optimized) Sensor Nodes and the 
%  paths between them (as defined by the Hoogeveen Path, and tangent/visbility graph).
function PlotObstacleSolutionGraph(SenseVarNodes)
global HoogeveenPath                          % contains target visitation order
global TargetNodes NumNodes ProblemD
global StartNode
%
SenseNodes = zeros(NumNodes,ProblemD);        % allocate space for sensing nodes array
SenseNodes(1,:) = StartNode;
for i=2:NumNodes                              % repackage sense node data in convenient way
    SVNindex = (i-2)*ProblemD + 1;
    SenseNodes(i,:)=SenseVarNodes(SVNindex:(SVNindex + ProblemD -1));
end
% display(SenseNodes);
    % Get path visitation order information from Hoogeveen Path
GraphTable = HoogeveenPath.Edges;         % get edge information as a MATLAB table
nedges = height(GraphTable);              % get the number of edges
fprintf(' There are %i edges in the Hoogeveen Path\n',nedges);
%% plot the edges between the optimized sensor node locations, including relevant obstacle vertices,
%  following the Hoogeveen Path
for i=1:nedges                            % loop over the edges
    CurEdge  = GraphTable{i,1};            % current edge to process
    CurIndex = CurEdge(1);  NextIndex = CurEdge(2);   % get indices of this edge
    CurNode  = SenseNodes(CurIndex,:);   NextNode = SenseNodes(NextIndex,:);
    fprintf('At step %i, processing edge (%i,%i)\n',i,CurIndex,NextIndex);
    fprintf('  CurNode = [%f,%f];  NextNode = [%f,%f]\n',CurNode(1),CurNode(2),NextNode(1),NextNode(2));
        % Must account for obstacles in plotting coverage path
    [edge_poly,weight] = ShortestNodePairPath(CurNode,NextNode);
    % line([CurNode(1),NextNode(1)],[CurNode(2),NextNode(2)],'Color','green','LineWidth',3);
    line(edge_poly{1},edge_poly{2},'Color','green','LineWidth',3);
    plot(CurNode(1),CurNode(2),'.','color','blue','MarkerSize',30);   % put dot at sensing locations
    plot(NextNode(1),NextNode(2),'.','color','blue','MarkerSize',30);   % put dot at sensing locations
    hold on;
end
%% plot the edges between the targets, including relevant obstacle vertices,
%  according to the Hoogeveen Path order
for i=1:nedges                            % loop over the edges
    CurEdge  = GraphTable{i,1};            % current edge to process
    CurIndex = CurEdge(1);  NextIndex = CurEdge(2);   % get indices of this edge
    CurNode  = TargetNodes(CurIndex,:);   NextNode = TargetNodes(NextIndex,:);
    fprintf('At step %i, processing Hoogeveen edge (%i,%i)\n',i,CurIndex,NextIndex);
    fprintf('  CurNode = [%f,%f];  NextNode = [%f,%f]\n',CurNode(1),CurNode(2),NextNode(1),NextNode(2));
        % Must account for obstacles in plotting coverage path
    [edge_poly,weight] = ShortestNodePairPath(CurNode,NextNode);
    % line([CurNode(1),NextNode(1)],[CurNode(2),NextNode(2)],'Color','green','LineWidth',3);
    line(edge_poly{1},edge_poly{2},'Color','magenta','LineWidth',3);
   % plot(CurNode(1),CurNode(2),'.','color','blue','MarkerSize',30);   % put dot at sensing locations
   % plot(NextNode(1),NextNode(2),'.','color','blue','MarkerSize',30);   % put dot at sensing locations
    hold on;
end

hold off;
end
