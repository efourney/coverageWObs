%% This function is for convenience: it plots the obstacles and the target node
%  locations, as might be needed for a sanity check on code development
function PlotObstacleProblemData()
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
end

