function [VG] = LeeVisibilityTree(Vertex)
global Obstacle_List nObst nObstacleVertices
%                                       
%
%% Construct the edges from the target nodes to the visibile obstacle vertices
%  We will assume that the angles are measured relative to the x-axis, and
%  measured relative to each target point
RawAngles = zeros(nObstacleVertices,3);
    % calcuate the angles subtendd by each obstacle vertex, and sort them
    % in ascending order
vcounter = 0;    
for i=1:nObst
    [bx,by]= boundary(Obstacle_List(i));        % boundary of ith obstacle
    nVertices = length(bx) - 1;                 % number of vertices of ith obstacle
    for j=1:nVertices
        vcounter = vcounter + 1;
        delta = [bx(j),by(j)] - Vertex;
        RawAngles(vcounter,1) = atan2(delta(2),delta(1));  % angle from Target to vertex
        RawAngles(vcounter,2)=j;                           % index of vertex
        RawAngles(vcounter,3)= i;                          % store the associated obstacle index
    end  % of j
end      % of i
    % sort vertex angles in ascending order
[~,idx] = sort(RawAngles(:,1));                 % sort just the vertex angles
Angles = RawAngles(idx,:,:);                    % now align the vertex id.s with the angles
fprintf('The Angle Matrix is:\n'); display(Angles);
%% Now start the process of "sweeping" through the vertices
OpenEdges = [];                                 % list of "open edges" that might affect visibility
x_axis_tip = Vertex + [1000,0];                 % end of line parallel to x-axis, starting at Vertex
    % check for intersection of the extended x-axis with the obstacle set
for i=1:nObst
    [bx,by] = boundary(Obstacle_List(i));       % get boundary of ith obstacle
    xaxis_x = [Vertex(1), x_axis_tip(1)];
    xaxis_y = [Vertex(2), x_axis_tip(2)];
    [x_int,y_int] = polyxpoly(xaxis_x,xaxis_y,bx,by,'unique');
    if ~isempty(x_int)
        if (length(x_int)==1)                   % intersect just a vertex of the ith obstacle
            printf('x-axis intersects a vertex of obstacle %i\n',i); % display(ii);
        else
            fprintf('x-axis intersects obstacle %i in multiple points\n',i);
            % display(ii);
        end
    end
end
% process each obstacle vertex to determine if it is visible   
end % of function