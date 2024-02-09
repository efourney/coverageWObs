%% Hoogeveen algorithm to find sub-optimal Path Order in case of Obstacles.
%  Takes input of a graph describing the target nodes, candidate edges, and 
%  the weights of the edges.  The output is a directed graph that describes
%  the Covering Path
%
function [PathGraph] = ObstacleHoogeveen(MaxGraph)
global NumNodes TargetNodes 
   % first find the minimum spanning tree
MST = minspantree(MaxGraph);                         % find the minimum spanning tree, MST 
   % now find the "wrong degree nodes."  These are the odd degree nodes for
   % nodes N2,...,N_(n-1), and the nodes s and/or t if they have even degree
match_nodes = [];                                    % list of nodes that need perfect matching
node_degrees = degree(MST);                          % list of all node degrees
odd_inner_nodes = find(mod(node_degrees(2:(NumNodes-1)),2));   % find the odd-degree nodes in N2,...,N(n-1)

adjusted_odd_inner_nodes = odd_inner_nodes + 1;      % indices are off by one, since we dropped the StartNode, GoalNode
%display(adjusted_odd_inner_nodes);

if find(adjusted_odd_inner_nodes)~= 0                % add any odd degree nodes to the matching list
    match_nodes = transpose([match_nodes,adjusted_odd_inner_nodes]);
end
   % next check the start and goal node degrees
if (mod(node_degrees(1),2)==0)                % if start node has even degree, it must be matched
    match_nodes = [match_nodes, 1];
end
if (mod(node_degrees(NumNodes),2)==0)         % if goal node has even degree, it must be matched
%    match_nodes = [match_nodes, (NumNodes+1)];  % ERROR (NumNodes+1) is WRONG--left over from days 
                                                 % when NumNodes didn't count for StartNode
     match_nodes = [match_nodes, NumNodes];                                            
end
%% now check to see if Perfect Matching is necessary
if length(match_nodes) == 0                           % no matches required 
    fprintf('ObstacleHoogeveen: no perfect matching required');
    MplusT = MST;                                     % the min span tree is the tour
else
    if mod(length(match_nodes),2) ==1
        fprintf('ObstacleHoogeveen:  ERROR--odd number of nodes to match\n');
        return;
    else                                      % if we get this far, matches are needed
            % find the minimum perfect matching, returned as cell array
        EdgeListCA = Obstacle_Min_Perfect_Match(match_nodes,MaxGraph);  
        Nedges = length(EdgeListCA);          % how many matched edges were found
        fprintf('ObstacleHoogeveen: Nedges (for matching)= %i\n',Nedges);
        MplusT = MST;                         % start with the MST
        for i=1:Nedges
            edge = EdgeListCA{i};             % an edge to be added to MST
            Node1 = TargetNodes(edge(1),:);    Node2 = TargetNodes(edge(2),:);
                % to find the edge weight between Node1 and Node2, we must
                % find shortest path
            edgeweight = MaxGraph.Edges.Weight(findedge(MaxGraph,edge(1),edge(2)));
            fprintf('matching edge between nodes %i and %i\n',edge(1),edge(2));
            MplusT=addedge(MplusT,edge(1),edge(2),edgeweight);  % add edge from perfect match to MST grap
        end
    end
end
[EulerGraph,exists] = Obstacle_Euler_Trail_STP(MplusT);        % find the Eulerian tour of MplusT
if (exists==0)
    fprintf('  Hoogeveen: problem in Directed Euler Trail STP\m');
end
%PathGraph  = CompleteHamiltonianShortcut(EulerGraph);
global testmode
testmode = false;
EulerGraph.Edges.Weight = [];
PathGraph  = Emily_HamiltonianShortcut(EulerGraph);
plot(PathGraph);
end
