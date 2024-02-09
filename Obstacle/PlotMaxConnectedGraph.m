%% This function is for convenience: it plots the obstacles, the target node
%  locations, and the Maximally connected Graph edges, as might be needed for 
%  debugging purposes. 
function PlotMaxConnectedGraph(MaxGraph)
global Obstacle_List
global SenseRadius

Locations = {};
%% first plot the obstacles
nObst = length(Obstacle_List);
for iplot = 1:nObst
    plot(Obstacle_List(iplot),'FaceColor','k');
    hold on;
end
%% now plot the target nodes and sensing region constraints
    % First extract the target nodes from MaxGraph
Locations = MaxGraph.Nodes.Location;      % locations of Target Nodes

nTargets = size(Locations);
TargetNodes = [];
for i=1:nTargets                          % sloppy code to convert Cell array to conventional array
    %CurNode = Locations{i};
    try
    TargetNodes(i,:) = Locations{i};      %
    catch
        TargetNodes(i,:) = MaxGraph.Nodes.Location(i,:);
    end
end
    % plot dots at TargetNodes
for iplot=1:nTargets
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    %viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
MaxGraphPaths = MaxGraph.Edges.Paths;
nPaths = length(MaxGraphPaths);
    % plot each connecting path
for i=1:nPaths
    CurPath = MaxGraphPaths{i};              % 1x2 Cell array, which each cell is a 1x2 arry of (x,y) coordinates of path point
    plot_x = CurPath{1};  plot_y = CurPath{2};
    plot(plot_x,plot_y);                     % plot line segment connecting these two points
    hold on;
end

