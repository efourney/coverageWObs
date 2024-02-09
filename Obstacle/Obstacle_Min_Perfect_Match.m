%% Find a minimum perfect matching on T-Join with an even number of nodes
% Mixed linear integer programming approach to finding the minimum perfect matching of
% the T-join nodes, which are are the nodes of the MST with odd-degree. 
% The function output, MatchList, is a cell array, with each array element a 
% 2-vector with the indices of the nodes that make the matching edge.
%
function [MatchList] = Obstacle_Min_Perfect_Match(Tjoin_Indices,MaxGraph)
global TargetNodes
%
N_T_Nodes = length(Tjoin_Indices);          % number of T-node indices to be matched

%% prepare the cost vector from the knowledge of T-join indices and target node locations.
x_length= N_T_Nodes * (N_T_Nodes -1)/2;     % number of components in the decision vector
CostVector = zeros(1,x_length); 
%
for i=1:(N_T_Nodes-1)
    for j=(i+1):N_T_Nodes
        x_index = (i-1)*N_T_Nodes - i*(i-1)/2 + (j-i);
        % fprintf('Tjoin_Indices i,j = %i, %i\n',Tjoin_Indices(i),Tjoin_Indices(j));
        Nodei = [TargetNodes(Tjoin_Indices(i),1),TargetNodes(Tjoin_Indices(i),2)]; 
        Nodej = [TargetNodes(Tjoin_Indices(j),1),TargetNodes(Tjoin_Indices(j),2)]; 
        CostVector(x_index) = MaxGraph.Edges.Weight(findedge(MaxGraph,Tjoin_Indices(i),Tjoin_Indices(j)));
        % CostVector(x_index) = norm(Nodei-Nodej);
    end
end
%% Make the Aeq beq equality constraint terms that enforce one edge per node in Tjoin_indices
beq = ones(N_T_Nodes,1);           % b vector in equality constraint is all ones
Aeq = zeros(N_T_Nodes,x_length);   % initialize Aeq matrix to zero
A = zeros(N_T_Nodes,x_length);     % these matrices are for inequality constraints.
b = zeros(N_T_Nodes,1);            % they are zeros because there are no inequalities 
                                       % constraint.  But MATLAB still needs these matrices
  % put constraint elements in the Aeq matrix
for i =1:N_T_Nodes                     % i indexes the rows of Aeq
    for j=1:(i-1)
        jmat = (j-1)*N_T_Nodes - j*(j-1)/2 + (i-j);
        Aeq(i,jmat) = 1;
    end
    for j=(i+1):N_T_Nodes
        jmat = (i-1)*N_T_Nodes - i*(i-1)/2 + (j-i);
        Aeq(i,jmat)=1;
    end
end
%% Call MATLAB's MILP solver    
intcon = transpose([1:x_length]);                 % indicates the integer variables
x_lb(1:x_length) = 0;                  % use lower/upper bounds to integrate binary variable
x_ub(1:x_length) = 1;
[X,fval] = intlinprog(CostVector,intcon,A,b,Aeq,beq,x_lb,x_ub);
%
%% Now match the solution to the original Tjoin_indices nodes
MatchList = cell(N_T_Nodes/2);         % create cell array to hold edge data
nmatch = 1;
for ixi=1:(N_T_Nodes-1)
    for jxi=(ixi+1):N_T_Nodes
        x_index = (ixi-1)*N_T_Nodes - ixi*(ixi-1)/2 + (jxi-ixi);
        if X(x_index)==1
            edge = [Tjoin_Indices(ixi);Tjoin_Indices(jxi)];
            MatchList{nmatch} = edge;
            nmatch = nmatch+1;
        end
    end
end 
