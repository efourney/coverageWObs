function Max_Graph = emMaxFroVis(TargetNodes,visbGraph)
n = length(TargetNodes(:,1)); 
%nodes 1-n in visb are of interest.
for j=1:n                           % annotate graph with node locations
    NodeLocations{j} = TargetNodes(j,:);
end
% fprintf('  Length of NodeLocations is %i\n',length(NodeLocations));
% display(NodeLocations);
Max_Graph = graph(zeros(n));
Max_Graph.Nodes.Location = transpose(NodeLocations);
Paths = {};
for i = 1:n-1
    for j = i+1:n
        [PATH,D] = shortestpath(visbGraph,i,j);
        myP = visbGraph.Nodes.Location(PATH,:);
        Weight = D;
        Paths{end+1} = {myP(:,1),myP(:,2)};
        Max_Graph = addedge(Max_Graph,i,j,D);
    end
end
Max_Graph.Edges.Paths = transpose(Paths);
end