%% "Obstacle_Global.m".  Contains global variable definitions and fixed 
%  paramter values related to shortest path with Obstacles, given single point targets 
% 
global Obstacle_List nObst            % list of obstacles, and number of obstacles
global ObstacleBoundary_List          % list of {bx,by} boundary elements for each obstacle
global ObstacleVertex_List            % list of [bx,by] vertex coordinates
global nObstacleVertices              % array which records number of vertices in each obstacle
global ObstacleVertexBase                 % array which records the "base" index of an obstacle's vertices
global TotalObstacleVertices          % total number of all obstacle vertices
global Obstacle_VG                    % visibility graph of obstacle set
global Obstacle_TG                    % tangent graph of obstacle set
global SObstacle                      % Array of binary variables that denote inter-target obstacles
%
global TargetNodes NumNodes           % n x 2 array of target nodes, number of target nodes
global ProblemD                       % dimension of workspace (in preparation for 3D future work)
global StartNode GoalNode             % the starting node
global Max_Graph                      % the maximally connected, weighted, graph of all Targets
global HoogeveenPath                  % sets the order of target visitation
global SenseRadius
% Optimize-related Global Variables
global NumContinuousVars              % number of continuous variables in Opti optimization
global FixedNodeIncidence             % incidence vector of Hoogeveen path-> needed for Opti
%
ProblemD = 2;                         % a planar environment and obstacles
SenseRadius = 0.5;                   % radius of circular sensing model
%SenseRadius = 1.3;                   % radius of circular sensing model
SenseRadius = 1.1;
    % Read_Obstacles;                     % get set of obstacles
obst1 = polyshape([0.0 2 4.5 2.0], [4.0 3 4.1 6.0],'KeepCollinearPoints',true);  
obst1 = polyshape([0.0 4.5 2.0], [4.0 4.1 6.0],'KeepCollinearPoints',true);  % shape of obstacle 1 (a triangle)
obst1 = polyshape(.2.*[0 0 2 3 5 5 3 2], 3+ .3.*[4 5 6 6 5 4 3 3],'KeepCollinearPoints',true); 

obst1 = polyshape(.6.*[4.6 5.2 5.2 5.5 5.5 5 5 4.6]+1.8,.85+.6.*[1.6 1.6 2 2 2.2 2.2 1.8 1.8],'KeepCollinearPoints',true); 
obst2 = polyshape([6.0 7.0 7.0 6.0], [1.0 1.0 5.0 5.0],'KeepCollinearPoints',true);   % shape of obstacle 2 (a quadrilaterial)
obst2 = polyshape([5.7,6.5,6.5,2.5,2.5,5.1,5.1,4.6,5.2,5.2,5.7], [0,0, 2.5,2.5,2,2,1.8,1.6,1.6,2,2],'KeepCollinearPoints',true);  
obst2 = polyshape([5.8 5.8 5.4 5.4 5.8 5.4 5.8 5.4 5.4 5.8 5.8 3.2 3.2 6.5 6.5], [0 .9 .9 1.1 1.1 1.3 1.5 1.5 1.7 1.7 2.25 2.25 2.5 2.5 0],'KeepCollinearPoints',true);  
obst2 = polyshape([5.8 5.8 5.4 5.4 5.8 5.8 5.4 5.4 5.8 5.8 3.2 3.2 6.4 6.4], [0 .9 .9 1.1 1.1  1.5 1.5 1.7 1.7 2.25 2.25 2.5 2.5 0],'KeepCollinearPoints',true);  
obst2 = polyshape([5.8 5.8 5.4 5.8 5.8 5.4 5.8 5.8 3.2 3.2 6.4 6.4], [0 .9 1 1.1  1.5 1.6 1.7 2.25 2.25 2.5 2.5 0],'KeepCollinearPoints',true);  
obst3 = polyshape([1 5.5 4 5.5], [2 4 2 1],'KeepCollinearPoints',true);

% obst3 = polyshape([1 5.5 4], [2 3 2],'KeepCollinearPoints',true);
obst4 = polyshape([5 5 5.5 5.5], [1.5 2 2 1.5],'KeepCollinearPoints',true);
obst4 = polyshape(2.7+ .2.*[0 0 2 3 5 5 3 2], 3.2+ .3.*[4 5 6 6 5 4 3 3],'KeepCollinearPoints',true); 
obst4 = polyshape([3.2 3.4 3.4 3.2],[1.4 1.4 2 2]-.05);
%obst3 = polyshape([1 5.5 4 5.5], [2 3 2 1],'KeepCollinearPoints',true);
obst3 = polyshape([2 3 3 4 4 3 3 2 2 1 1 2], [-1 -1 1 1 2 2 3 3 2 2 1 1],'KeepCollinearPoints',true);
% obst3 = polyshape([2 3 4 4 3 2 1 1], [-1 -1 1 2 3 3 2 1],'KeepCollinearPoints',true);
 obst3 = polyshape([2.5 2.5 5 5 3.3 3.3 3.8 3.8 4.5 4.5 2.9 2.9], [-1 1 1 -1 -1 .1 .1 -.5 -.5 .5 .5 -1],'KeepCollinearPoints',true);
  obst3 = polyshape([2.5 2.5 5 5 3.3 3.3 3.8 3.8 4.8 4.8 2.9 2.9], [-1 1 1 -1 -1 .1 .1 -.5 -.5 .8 .8 -1],'KeepCollinearPoints',true);
  obst3 = polyshape([3.45 5.2 5.2 5 5 3.45]-.25, [1 1 .4 .4 .8 .8 ]+.1,'KeepCollinearPoints',true);
Obstacle_List = [obst3,obst1,obst2,obst4]; 

o1 = polyshape([0 0 10 10 8 8 9 9 5.8 5.8 6 6 5 5 5.2 5.2 1 1 2 2],[0 10 10 0 0 1 1 9 9 5 5 4 4 5 5 9 9 1 1 0],'KeepCollinearPoints',true); 
o2 = polyshape([-1 11 11 2 2 -1 -1 3 3 10 10 -1],[12 12 20 20 15 15 14 14 19 19 13 13]);
o3 = polyshape([5 6 6 5],[14 14 18 18]);
o4 = polyshape([7 8 8 7],[14 14 18 18]);
%Obstacle_List = [o1,o2,o3,o4]; 
% list of obstacles
boundingBox = [-3,-3;-3,30;30,30;30,-3];
boundingBoxTrue = boundingBox;


    % Prepare basic obstacle set information
nObst = length(Obstacle_List);            % number of obstacles
ObstacleBoundary_List = [];               % allocate space for list of obstacle boundaries
ObstacleVertex_List = [];                 % allocate space for list of obstacle boundary vertices

nObstacleVertices = [];                   % allocate space for number of vertices on each obstacle boundary
VertexAccumulate = 0;
TotalObstacleVertices = 0;
for i=1:nObst
    nVert_i = numsides(Obstacle_List(i));             % number of vertices in obstacle i
    TotalObstacleVertices = TotalObstacleVertices + nVert_i;
    nObstacleVertices(i) = nVert_i;
    ObstacleVertexBase(i) = VertexAccumulate;         % index of lowest index of ith obstacle vertex list
    VertexAccumulate = VertexAccumulate + nVert_i;
end
    % Read TargetNodes
TargetNodes = [-1.0 -1.0; 1.0 0; 4.0 0.0; 4.8 1.3; 4.0 3.5; 0.0 5.0; 6.0 6.0; 8.0 3.0; 8.0 5.0; 7.0 0.0];

TargetNodes = [3.7 1.7; 2.9 1.7; 5.7,2; 4.8 1.3;3.8,1.1; 5.5,.2; 4,.5; 6.0 6.0; 8.0 3.0; 8.0 5.0; 12 0.0];

TargetNodes = [4,.5; 4.46,1.93;  5.7,1.75; 5.65, 1.3; 5.5,.2; 3.8,1.15; 2.7,1.1];

% TargetNodes = [-2, 13.5;
%     -.5, 13.1;
%     2,13.7;
%     4.8,14.4;
%     4,15;
%     4.5,16;
%     4.1,14.7
%     4,18.5;
%     5,18.1;
%     4.8,18.4;
%     5.1,18.2;
%     6.5,15;
%     6.2,16;
%     6.3,14.7;
%     6,13.5;
%     7,13.3;
%     9,13.5;
%     9,15;
%     9.7,16;
%     8.1,15.7;
%     8.3,15.1;
%     9.5,14.1;
%     9,14;
%     9,18.5];
% TargetNodes = [0.0 0.0; 1.5 1.2; 1.5 0; 3.0 3.0; 2.0 2.0];
StartNode = TargetNodes(1,:);      GoalNode = TargetNodes(end,:);
NumNodes = length(TargetNodes);
NumContinuousVars = ProblemD * (NumNodes -1);


