%%  Function that finds "shortcuts" that turn a (DIRECTED) graph that satisfies 
%   the Euler trail property (each graph edge visited only once) into a
%   Directed Hamltonian Path (each vertex visited only once) by "shortcutting" the
%   Euler Graph (which is the input, EG)
%
%   This algorithm is currently funky because Matlab's graph representation
%   scheme and graph manipulation algorithms are sub-optimal.  Moreover,
%   this algorithm does not yet try any greedy optimization of the choice
%   of cuts in the typical situation where there are multiple cut choices.
%   Also, there is a HACK in the current implementation (see below).
%   Search for "HACK"
%
function [HGraph] = CompleteHamiltonianShortcut(EulerGraph)
global Max_Graph
global NumNodes
%
HGraph = EulerGraph;                                   % copy graph, because it will be modified
    %% First Check Start Node, and Modify as needed. HACK: this current code
    %  stops when it finds the first relinking process that doesn't cause a
    %  disconnected graph.  It would be better to check all possible
    %  relinkings, and choose the lowest cost one.
if (indegree(HGraph,1) ~=0)                    % check if incoming edges on StartNode, which must be relinked
    StartPre = predecessors(HGraph,1);         % get predecessors, successors of StartNode
    StartPost = successors(HGraph,1);             
    for i=1:length(StartPre)                   % must relink every incoming edge
        HGraph = rmedge(HGraph,StartPre(i),1); % remove the incoming link
        for j=1:length(StartPost)              % try all possible outgoing relinks
            HGraph = rmedge(HGraph,1,StartPost(j));  % remove outgoing edge to InPost(j)
            edgeweight = Max_Graph.Edges.Weight(findedge(Max_Graph,StartPre(i),StartPost(j)));
            HGraph = addedge(HGraph,StartPre(i),StartPost(j),edgeweight);
            if (~disconnected(HGraph))
                break;
            end
        end  % of for j
    end      % of for i
end          % of if (Instart ~=0);
    %% Next Check GoalNode, and Modify as needed. HACK: this current code
    %  stops when it finds the first relinking process that doesn't cause a
    %  disconnected graph.  It would be better to check all possible
    %  relinkings, and choose the lowest cost one.
if (outdegree(HGraph,NumNodes) ~=0)            % check if there outgoing edges on GoalNode, which must be relinked
    GoalPre = predecessors(HGraph,1);          % get predecessors, successors of GoalNode
    GoalPost = successors(HGraph,1);             
    for i=1:length(GoalPost)                   % must relink every OutGoing edge
        HGraph = rmedge(HGraph,NumNodes,GoalPost(i));    % remove the OutGoing link
        for j=1:length(GoalPre)                % try all possible InComing relinks
            HGraph = rmedge(HGraph,1,GoalPre(j));  % remove outgoing edge to InPost(j)
            edgeweight = Max_Graph.Edges.Weight(findedge(Max_Graph,GoalPost(i),GoalPre(j)));
            HGraph = addedge(HGraph,GoalPost(i),GoalPre(j),edgeweight);
            if (~disconnected(HGraph))
                break;
            end
        end  % of for j
    end      % of for i
end          % of if (Instart ~=0);

    %% Now check if interior links need to be modified (shortcut).  Note that we 
    %  calculate the in and out degrees AFTER the above processing.  Any
    %  interior node with even degree and degree >2 must be relinked.
InD = indegree(HGraph);  OutD = outdegree(HGraph);  % get in an out degrees of graph
d = InD + OutD;                                % degrees of the graph nodes
ProcessNodes = find((mod(d,2)==0) & (d>3));    % find nodes with even degree and degree>3
nProcess = length (ProcessNodes);              % number of nodes to process
    %  If nProcess>0, then there are nodes to process. Else, return the input as is
if (nProcess > 0)                              
    fprintf('Hamiltonian ShortCut: there are nodes to process\n');
    for i=1:nProcess                               % cycle through nodes with high even degree
        CurNodeIndex = ProcessNodes(i);            % get the next candidate node
        before = predecessors(HGraph,CurNodeIndex);    % get predecessor node array
        after = successors(HGraph,CurNodeIndex);       % get successor node array
        numpaths = length(before);                     % number of paths passing through
            % process the multiple paths passing through the current node
        CutDone = false;                           % boolean to control while loop
        CutFound = false;                          % boolean to indicate if acceptable cut is found
        jcut = 1;                                  % counter over possible paths
        while ~CutDone
                % Try the first cut
            TempGraph = ByPass(HGraph,before(jcut),ProcessNodes(i),after(jcut));
                % check if the graph has been disconnected.  If so, then
                % try alternative cut.
                % HACK: the current code only tries one alternative.  This
                % should be rewritten to try all alternatives.
            if disconnected(TempGraph)
                fprintf('           Anamolous disconnection of graph\n');
                    % shift pairings if predecessor/successor nodes to find
                    % another cut option
                after = circshift(after,1);    % change the before/after pairing
                TempGraph = ByPass(HGraph,before(jcut),ProcessNodes(i),after(jcut));
                fprintf('      Trying ShortCut: path %i with sequence [%i,%i,%i]\n',jcut,before(jcut),ProcessNodes(i),after(jcut));
                    % if the alternative is disconnected, then we're done
                    % the first set of paths.  Update jcut, and try another set.
                if disconnected(TempGraph)
                    fprintf('       Alternative graph is disconnected\n');
                    jcut = jcut +1;
                else
                    fprintf('       Alternative graph successful\n');
                    CutDone = true;
                    CutFound = true;
                    HGraph = TempGraph;
                end
            else       % if the first cut was not disconnected, pass back the solution
                HGraph = TempGraph;
                CutDone = true;
                CutFound = true;
            end
            if (jcut>=numpaths)   % did we exhaust all paths?
                CutDone = true;
            end
                % If we couldn't find an non-disconnected cut, ABORT
            if ~CutFound
                fprintf('ERROR:     ShortCut Anamoly!\n');
            return;
            end
        end % of while
    end % of for loop
end % if (nProcess>0)
end   % function

%% Local function to change the links in a directed graph to implement a Hamiltonian cut
%  Because this gets called multiple times, it made sense to put this a
%  small function
function [NewGraph] = ByPass(G,pre,node,post)
NewGraph = rmedge(G,pre,node);
NewGraph = rmedge(NewGraph,node,post);
NewGraph = addedge(NewGraph,pre,post,1);
end

%% Local function to check if a graph is disconnected.  This function is REALLY
%  inefficient due to MATLAB's way of dealing with graphs.  It is easy to
%  check if an undirected graph is disconnected by using the conncomp function.  
%  BUT, conncomp applied to a directed graph divides the graph into
%  components that are all connected by TWO way connections.  Hence, we
%  convert the directed graph temporarily to an undirected graph, and then
%  check for discontinuity.  Perhaps there is a more efficient way in MATLAB?
%
function [status] = disconnected(G)
[s,t]=findedge(G);
unG = graph(s,t);
partvect = conncomp(unG) - 1;          
if find(partvect)>0
    fprintf('ERROR:  Graph is disconnected in Hamiltonian ShortCut\n\n');
    status = true;                              % exists=false is maintained;
else
    status = false;
end
end