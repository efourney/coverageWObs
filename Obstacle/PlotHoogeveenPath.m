%% This function is for convenience: it plots the obstacles and the target node
%  locations, as might be needed for a sanity check on code development
function PlotHoogeveenPath(HDiGraph)
global Obstacle_List
global TargetNodes
global SenseRadius

%% first plot the obstacles
nObst = length(Obstacle_List);
for iplot = 1:nObst
    plot(Obstacle_List(iplot));
    hold on;
end
%% now plot the target nodes and sensing region constraints
[nTargets,dummy] = size(TargetNodes); 
    % plot the start node and its sensing circle 
% plot(StartNode(1),StartNode(2),'.','color','red','MarkerSize',20);
% hold on;
    % plot sensing range as circles
% viscircles(StartNode,SenseRadius,'LineStyle','--');
% now plot the same for the other target nodes
for iplot=1:nTargets
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    hold on;
        % plot sensing range as circles
    viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
% Now Plot the Hoogeveen Path, which is a cell array, where each cell
% contains the (x,y) coordinate of the sensing node in the Hoogeveen path
% order solution.
HPathNodes = table2array(HDiGraph.Nodes);
%display(HPathNodes);
HPathLength = length(HPathNodes);
for i=1:(HPathLength-1)
    CurPoint = HPathNodes{i};            % 1x2 Cell array, which each cell is a 1x2 arry of (x,y) coordinates of path point
    NextPoint = HPathNodes{i+1};
    plot_x = [CurPoint(1),NextPoint(1)];  plot_y = [CurPoint(2),NextPoint(2)];
    plot(plot_x,plot_y);                % plot line segment connecting these two points
    hold on;
end
end  % of function

