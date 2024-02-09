%% function that optimizes the sensing node locations, in the presence of obstacles,
%  given a fixed node visitation order.
%  INPUT:  a list of initial Sensing Node locations.  
%  OUTPUT: a list of optimized sensing node locations
%  NOTE: sensing node 1 is fixed to the same location as the starting node.
%
function [SensingNodes] = ObstaclePathOptimization(SenseNodes0)
global NumNodes 
global NumContinuousVars
global SenseRadius
%
fun = @ObstaclePathCost;                          % get pointer to the Path Cost function
nlcon = @ObstacleSenseConstraint;                 % nonlinear sensing constraint function 
ConstraintUpper(1:(NumNodes-1)) = SenseRadius^2;  % right hand side of nonlinear state constraint
   % some solvers need bounds to operate.  Because there are no natural bounds on the 
   % continous variables, these bounds are arbitrary.  The should be considered carefully, 
   % to make sure they do not eliminate actual solutions
LowBound(1:NumContinuousVars)= -3.0;
UpBound(1:NumContinuousVars) = 10.0;

%% set up the optimization framework
% the xtype string describes continuous (C), integer(I), or binary(B) variables. 
xtype = '';
for i=1:NumContinuousVars
    xtype=append(xtype,'C');
end

% Options for the Opti solver.  The argument after 'solver' dictates which solver to run.
% 'display' controls what info is displayed during the solving process.  For now, solver 
% must be: 'nomad', 'bonmin','SCIP'.  I've found that only BONMIN work properly
opts = optiset('solver','bonmin','maxiter',20);

% Now the optimization is to minimize the total path distance, with the
% constraint that the sensing locations lie inside the sensing regions (an
% inequality constraint
Opt = opti('fun',fun,'nlcon',nlcon,'cu',ConstraintUpper,'bounds',LowBound,UpBound,'xtype',xtype,'options',opts);

[SensingNodes,fval,exitflag,info] = solve(Opt,SenseNodes0);
switch exitflag
    case 1
        fprintf('  Optimization: exitflag = 1\n');
    case 0  
        fprintf('  Optimization: Optimal Solution Found\n');
    case -1 
        fprintf('  Optimization: Infeasible Solution\n');
    case -2
        fprintf('  Optimization: Unbounded Solver Error\n');
    case -3
        fprintf('  Optimization: Solver Specific Error\n');
    case -5
        fprintf('  Optimization:  User Criteria met\n');
end 
OptCounter = 1;
end 