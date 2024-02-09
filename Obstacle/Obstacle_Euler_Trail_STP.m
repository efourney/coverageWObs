%% Given an UNDIRECTED graph, G, this function constructs a DIRECTED Graph 
%  representing an Euler Tour from the input graph (assuming that the graph 
%  has the properties needed for the Euler processing). The graph is returned 
%  in the variable GEuler.  The flag "exists" is false (0)
%  if no tour exists, or true (1) if a tour exists.  This code is based on 
%  Fleury's algorithm, which is relatively easy to understand, but is not
%  as efficient as Hierholzer's algorithm.  MATLAB's mediocre method for
%  representing graphs also makes this code inefficient and funky.
%
function [GEuler, exists] = Obstacle_Euler_Trail_STP(G)
%
n = numnodes(G);                     % how many nodes in G?
d = degree(G);                       % list of all node degrees
dparity = mod(d,2);                  % find odd and even degree nodes
NumOddDegreeNodes = sum(dparity);    % find the number of odd degree nodes
exists = false;
%% error checking: make sure the graph has no anomalies
if sum(NumOddDegreeNodes) > 2        % if more than 2 odd-degree nodes, then no tour exists
    fprintf('ERROR in Euler Trail: more than 2 odd degree nodes\n\n');
    ResultGraph = graph([],[]);      % Return empty graph, and maintain exists=false
    return;
end
% if there are no edges in the graph, return the graph, and maintain exists=false, too
if numedges(G) == 0                 % if there are no edges, then tour impossible
    fprintf('ERROR in Euler Trail: no edges in graph\n\n');
    ResultGraph = G;
    return;
end
% we have to make sure there is exactly one nontrivial graph component.  conncomp(G)
% is a vector, with an integer for each node which describes the connected
% components to which that integer belongs. If only one connected component, then 
% partvect is a vector of zeros
partvect = conncomp(G) - 1;          
if find(partvect)>0
    fprintf('ERROR:  Graph is disconnected in Euler Trail\n\n');
    GEuler = G;                              % exists=false is maintained;
    return;
end
%% Now find the Eulerian tour, knowing that the graph has no basic anomalies
%  Where do we start? if there are two odd vertices, we must start on one of those.
%  otherwise, start at the first labeled vertex (but you could start anywhere)
if NumOddDegreeNodes == 2                % case of two odd vertices
    OddNodes = find(dparity);            % get indices of odd nodes.               
    start = OddNodes(1);                 % choose  the first indexed odd degree vertex 
else 
    if NumOddDegreeNodes == 1            % check for anomaly
        fprintf('ERROR in Euler Trail:  single odd degree node\n');
        return;
    end
    start = 1;                           % by default, start at first node.  Alternatively, the start could a random node
end
v = start;                               % begin at start node determined above.

%% main part of the algorithm (This if Fleury's algorithm)
     % prepare to start the tour
m = numedges(G);                              % number of graph edges
h = G;                                        % copy the graph, since edges are removed 
                                              %   in Fleury's algorithm
elist = zeros(m,2);                           % space to store edge traversal sequence
step = 0;
% 
while numedges(h)                             % as long as h has any edges
    step = step+1;
    next = neighbors(h,v);                    % get neighbors of v
    nlength = length(next);
    vdegree = degree(h,v);                    % get degree of v: possible multiple edges to nhbr
        % if v has even degree (or only one neighbor) any neighbor will do
    if (mod(vdegree,2) == 0) | (vdegree == 1)
        w = next(1);
    else                                  % process all of the neighbors to see if one 
                                          %   can be removed with disconnecting graph
        FoundEdge = false;                % boolean needed to control the next while loop
        wcounter = 1;                     % counter needed to determine if we must exit while
        while ~FoundEdge 
            w = next(wcounter);           % get the next node in the neighbor list
            if (degree(h,w) == 1)         % this is a bridge node: don't remove edge
                wcounter = wcounter + 1;
            else                          % if node is not a bridge, then go to edge removal step
                htest = my_rmedge(h,v,w);     % remove a single edge
                components = conncomp(htest);  % get labels on connected subgraphs of h
                    % check if v,w still have a path
                if (components(v) == components(w))
                    FoundEdge = true;
                else
                    wcounter = wcounter + 1;
                end
            end      % if (degree(h,w) == 1
            if (wcounter > nlength)        % if we found no edges that can be successfully removed
                fprintf('ERROR: Directed Euler Trail: all edge removals cause disconnection\n');
                return;
            end
        end          % end while ~FoundEdge
    end
    h = my_rmedge(h,v,w);              % Note, Matlab removes ALL edges from v,w
    elist(step,:) = [v,w];
    v = w;
end
%% Now construct (from elist) a DIRECTED graph that represents the Euler Trail.
%  After constructing the graph, we must add back the edge weights, node locations, 
%  and overlap information . There should be a more efficient way to store and reallocate
%  the weights and overlap info from the input graph to the new directeed graph.  BUt
%  for now we have this ugly code that recalculates these quantities. 
[nlist,wlist] = size(elist);                      % nlist is the number of edges
s = [];  t= [];  edgeweights = [];  Locations = {};  
for i=1:nlist
    Node1 = elist(i,1);  Node2 = elist(i,2);      % two nodes on either end of the edge
    s = [s, Node1];      t = [t,Node2];
    sLocation = h.Nodes.Location{Node1};  tLocation = h.Nodes.Location{Node2};
    edgeweights(i) = norm(tLocation - sLocation); % recalculate weight
    Locations{Node1} = sLocation;                 % assignthe node's cartesian location
    Locations{Node2} = tLocation;
end
    % now construct the new DIRECTED graph, and annotate it
GEuler=digraph(s,t,edgeweights);
GEuler.Nodes.Location = transpose(Locations);
exists = true;
end   %function

%% local function that accounts for the fact that if nodes (v,w) have multiple edges 
%  connecting them, MATLAB removes all of them when you call rmedge()
function [newgraph] = my_rmedge(h,v,w)
%
NumEdges = edgecount(h,v,w);                 % find out if more than one edge between v,w
h = rmedge(h,v,w);                           % Note, Matlab removes ALL edges from v,w
if (NumEdges > 1)                            % if there were multiple edges, restore all but 1
    wNode = h.Nodes.Location{w};   
    vNode = h.Nodes.Location{v};
    edgeweight = norm(wNode-vNode);
    fprintf('            MyRMedge:  edgeweight = %f \n',edgeweight);
    for k = 1:(NumEdges-1)
        newgraph = addedge(h,v,w,edgeweight);
    end     
else
% newgraph = rmedge(h,v,w);
  newgraph = h;
end
end