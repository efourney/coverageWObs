%% plot index numbers of the triangles and edges on the plot itself
function [] = PlotTriangulationIndices(T)
%global TriangulationEdges
%
TriangulationEdges = edges(T);
IC = incenter(T);
numtri = size(T,1);
TriLabels = arrayfun(@(P) {sprintf('T%d', P)}, (1:numtri)');
Htl = text(IC(:,1),IC(:,2),TriLabels,'FontWeight','bold', ...
'HorizontalAlignment','center','Color','blue');
hold on;

%% plot index numbers of the triangles on the plot itself
NumEdges = size(TriangulationEdges,1);
EdgeLabels = arrayfun(@(P) {sprintf('e%d', P)}, (1:NumEdges)');
EC = [];
for i=1:NumEdges
    curEdge = TriangulationEdges(i,:);
    EC(i,:) = T.Points(curEdge(1),:) + 0.5*(T.Points(curEdge(2),:)-T.Points(curEdge(1),:));
end
Htl = text(EC(:,1),EC(:,2),EdgeLabels,'FontWeight','bold', ...
'HorizontalAlignment','center','Color','black');
hold on;
end    % of function