%% plot the tangent graph of the targets --to check their validity
%  We assume that each node is annotated with its Cartesian coordinates,
%  which are used in the MATLAB "plot" function below
function [p_return] = PlotObstacleTangentGraph(VG)
global Obstacle_List
global Obstacle_VG
figure
% First plot the obstacles
nObst = length(Obstacle_List);  
for i=1:nObst
    plot(Obstacle_List(i));
    hold on
end
% Now plot the vertices and the edges
nVertices = numnodes(VG);                            % number of nodes in visibility graph
VertexLocations = VG.Nodes.Location;                % Cartesian Coordinates of each node
Xdata =[]; Ydata = [];                               % space for node locations
for i=1:nVertices
    CurNode = VertexLocations(i,:);
    Xdata(i) = CurNode(1); Ydata(i) = CurNode(2);
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    hold on;
end
% VG.XData = Xdata;  VG.YData = Ydata;
p_return = plot(VG,'EdgeColor','b','NodeColor','r','XData',Xdata,'YData',Ydata);
end  % of function