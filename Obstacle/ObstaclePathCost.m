%% function to compute the cost of a coverage path, given a chosen node visiting
%  order, including the effects of obstacles. The data passed to this function 
%  consists of the locations of the "active" sensing nodes, i.e., the ones whose 
%  positions can be optimized.  It is passed in a "flat" form: a 1 x (ProblemD *
%  (number of active nodes)), since this is the format required for the
%  Opti optimization toolkit
%%
function[sum] = ObstaclePathCost(CurSenseNodes)
global StartNode NumNodes ProblemD
global FixedNodeIncidence                % the target visiting order, in incidence vector form
global SObstacle                         % Boolean that indicates if obstacles might interfere 
                                         %    a with the straight path between sensor nodes 
sum=0.0;
    % since s1 is fixed to the StartNode location, find cost from StartNode to 
    % the first variable sensing node (Sj ,ffor some j=2,...,n)
for j=1:(NumNodes-1)
    if FixedNodeIncidence(j) == 1                   % check to see if link is active
        jindex = (j-1) * ProblemD + 1;              % array index into "flat" Sensor Node vector
        sensenodej = CurSenseNodes(jindex:(jindex+ProblemD-1));
            % check if the straight line shortest path solution can be
            % used, or if the more complicated visbility graph solution
            % must be used.
        if (SObstacle(1,j) == false)
            sum = sum + norm(sensenodej - StartNode);
        else
            [node_pair_polyline,weight] = ShortestNodePairPath(StartNode,sensenodej);
            sum = sum + weight;
        end
    end
end
    % now compute contributions from all other edges
for i=1:(NumNodes-2)
    isense = (i-1) * ProblemD + 1;    
    si = CurSenseNodes(isense:(isense+ProblemD-1));
    for j=(i+1):(NumNodes-1)
        xi_arg = i * NumNodes - i*(i+1)/2 + (j-i);
        if FixedNodeIncidence(xi_arg) == 1              % check to see if link i-to-j is active
            jarg = (j-1) * ProblemD + 1;
            sj = CurSenseNodes(jarg:(jarg+ProblemD-1));
                % see if straight line or visibility graph length is needed
            if (SObstacle(1,j) == false)
                sum = sum + norm(si-sj); 
            else
                [node_pair_polyline,weight] = ShortestNodePairPath(si,sj);
                sum = sum + weight;
            end  % of if
        end   % of if
    end   % of j=
end   % of for i=
fprintf('ObstaclePathCost: %f\n',sum);
end   % of function

