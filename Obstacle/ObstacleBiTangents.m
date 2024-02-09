%% this function computes the 4 bitangents (2 separating, 2 supporting) for a pair
%  of convex obstacle polygons that are identified by their indices obst_i and obst_j
%
function [BiTangents] = ObstacleBiTangents(obst_i,obst_j)
global Obstacle_List
global ObstacleBoundary_List ObstacleVertex_List
%
BiTangents = zeros(4,2);                           % allocate space for bitangent vertex array
%% first find a pair of "facing" edges (or vertices), one in each obstacle. This
%  pair is needed for the subsequent iterations
bx_i = ObstacleBoundary_List{obst_i}{1};  by_i = ObstacleBoundary_List{obst_i}{2};
bx_j = ObstacleBoundary_List{obst_j}{1};  by_j = ObstacleBoundary_List{obst_j}{2};  
nVert_i = length(bx_i)-1;   nVert_j = length(bx_j)-1;      % number of obstacle vertices
    % compute the centroid of each obstacle
centroid_i_x = sum(bx_i(1:nVert_i))/nVert_i;  centroid_i_y = sum(by_i(1:nVert_i))/nVert_i;
centroid_j_x = sum(bx_j(1:nVert_j))/nVert_j;  centroid_j_y = sum(by_j(1:nVert_j))/nVert_j;
% plot(Obstacle_List(obst_i)); 
% hold on; 
% plot(Obstacle_List(obst_j));
% plot(centroid_i_x,centroid_i_y,'.','color','red','MarkerSize',20); hold on;
% plot(centroid_j_x,centroid_j_y,'.','color','red','MarkerSize',20); hold on;
    % intersect the line connecting the two centroids with the obstacle boundaries.
    %  This process will find two edges that are "facing each other".
midline_x = [centroid_i_x,centroid_j_x]; midline_y = [centroid_i_y,centroid_j_y]; 
% plot(midline_x,midline_y);
[int_i_x,int_i_y,ii_i] = polyxpoly(midline_x,midline_y,bx_i,by_i);  % intersection with obstacle i
[int_j_x,int_j_y,ii_j] = polyxpoly(midline_x,midline_y,bx_j,by_j);  % intersection with obstacle i

% extract the "edge index" from each obstacle.  ii_i(2) is the edge on
% the ith obstacle that is intersected, etc. 
edge_i = ii_i(2);  edge_j = ii_j(2);              % edge index of intersection edges
% plot(bx_i(edge_i),by_i(edge_i),'.','color','green','MarkerSize',20); hold on;
% plot(bx_i(edge_i+1),by_i(edge_i+1),'.','color','green','MarkerSize',20); hold on;
% plot(bx_j(edge_j),by_j(edge_j),'.','color','green','MarkerSize',20); hold on;
% plot(bx_j(edge_j+1),by_j(edge_j+1),'.','color','green','MarkerSize',20); hold on;
%% Now construct the bitangents
    % First visualize the initial 4 eges, consisting of the "separating" or "cross"
    % tangents, then the "supporting"% tangents.
% CrossLine1_x = [bx_i(edge_i),bx_j(edge_j)]; CrossLine1_y = [by_i(edge_i), by_j(edge_j)];
%CrossLine2_x = [bx_i(edge_i+1),bx_j(edge_j+1)]; CrossLine2_y = [by_i(edge_i+1), by_j(edge_j+1)];
%line(CrossLine1_x,CrossLine1_y,'Color','black','LineWidth',2);
%line(CrossLine2_x,CrossLine2_y,'Color','black','LineWidth',2);
%
%SupportLine1_x = [bx_i(edge_i), bx_j(edge_j+1)];  SupportLine1_y = [by_i(edge_i), by_j(edge_j+1)];
%line(SupportLine1_x,SupportLine1_y,'Color','black','LineWidth',2);
%SupportLine2_x = [bx_i(edge_i+1), bx_j(edge_j)];  SupportLine2_y = [by_i(edge_i+1), by_j(edge_j)];
%line(SupportLine2_x,SupportLine2_y,'Color','black','LineWidth',2);
%
[vi,vj] = MakeBiTangent(obst_i,edge_i,obst_j,edge_j,-1,-1);
BiTangents(1,:) =[vi,vj];
[vi,vj] = MakeBiTangent(obst_i,edge_i,obst_j,mymod((edge_j+1),nVert_j),-1,+1);
BiTangents(2,:) =[vi,vj];
[vi,vj] = MakeBiTangent(obst_i,mymod((edge_i+1),nVert_i),obst_j,edge_j,1,-1);
BiTangents(3,:) =[vi,vj];
[vi,vj] = MakeBiTangent(obst_i,mymod((edge_i+1),nVert_i),obst_j,mymod((edge_j+1),nVert_j),1,1);
BiTangents(4,:) =[vi,vj];
end    % of function

