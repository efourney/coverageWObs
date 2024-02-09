%% Helper function "IsVertexTangent."  This function returns a boolean that determines
%  if the line from a Vertex V (which might be on the boundary of an obstacle ) 
%  to Vertex Z (specified by its index) on the obstacle (specified by OIndex) 
%  is tangent at Vertex Z.  
function [Tangent] = IsVertexTangent(V,OIndex,ZIndex)
global ObstacleBoundary_List ObstacleVertex_List
%
% fprintf('IsVertexTangent: OIndex = %i, Zindex = %i\n',OIndex,ZIndex)
vx = ObstacleVertex_List{OIndex}{1};
vy = ObstacleVertex_List{OIndex}{2};
% bx(end) = [];  by(end) = [];  nVert = length(bx);
nVert = length(vx);
    % construct 3 obtacle vertices (at Zindex, and the vertices
    % before and after) needed for testing tangency
Zvertex = [vx(ZIndex),vy(ZIndex)];
Zp1_index = mymod((ZIndex+1),nVert);  
Zm1_index = mymod((ZIndex-1+nVert),nVert);
Zp1 = [vx(Zp1_index),vy(Zp1_index)];    Zm1 = [vx(Zm1_index),vy(Zm1_index)];
if (IsLeft(V,Zvertex,Zp1) > 0)
    if (IsLeft(V,Zvertex,Zm1) > 0)
        Tangent = true;
    else
        Tangent = false;
    end
else
    if (IsLeft(V,Zvertex,Zm1) < 0)   
        Tangent = true;
    else
        Tangent = false;
    end
end     % of if IsLEFT
end     % of function

%% The following functions are taken from, and converted from C++ to MATLAB
%  http://geomalgorithms.com/a15-_tangents.html
%% Helper function "IsLeft" to determine if point P2 is "to the left" of
%  the line extending from P0 to P1, for (P0,P1,P2) being points the plane.
%     INPUTS: points P0, P1, P2
%     OUTPUT: This function returns a scalar real number with the interpretation:  
%             >0 if P2 is on left of line from P0 to P1
%             =0 if P2 is on the line from P0 to P1
%             <0 if P2 is right of line from P0 to P1
%  
function [value] = IsLeft(P0,P1,P2)
    value = (P1(1) - P0(1)) * (P2(2)-P0(2)) - (P2(1) - P0(1)) * (P1(2) - P0(2));
end  % of function IsLeft
