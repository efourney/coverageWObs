%% Now find Bitangent by moving vertices around obstacle boundaries until bitangent condition
%  is met.  
function [itan_index,jtan_index] = MakeBiTangent(obst_i,vertex_i,obst_j,vertex_j,increment_i,increment_j)
global ObstacleBoundary_List ObstacleVertex_List
    % get obstacle boundary elements
Obdy_i = ObstacleVertex_List{obst_i};  vx_i = Obdy_i{1}; vy_i = Obdy_i{2};
Obdy_j = ObstacleVertex_List{obst_j};  vx_j = Obdy_j{1}; vy_j = Obdy_j{2};
nVert_i = length(vx_i);  nVert_j = length(vx_j);
    % Iterate vertices as needed to result in BiTangent 
iIndex = vertex_i;  jIndex = vertex_j;    
iFixed= true;                         % start by processing the jth object end of temporary tangent 
BothTangent = false;
    % now iterate back-and-forth on vertices of the two obstacles
while (BothTangent==false)
    if iFixed                         % if it is turn for obstacle i to have fixed vertex
            % check if vertex on obstacle j is "tangent"  
        if IsVertexTangent([vx_i(iIndex),vy_i(iIndex)],obst_j,jIndex)==true  
                % if so, then check if vertex on obstacle i is also tangent
            if IsVertexTangent([vx_j(jIndex),vy_j(jIndex)],obst_i,iIndex)==true
                BothTangent = true;   % since both are tangent, we're done
            else                      % need return to moving vertex on obstacle i 
                iFixed = false;
            end
        else                          % since vertex on obst j not tangent, iterate until it is
            jIndex = mymod((jIndex + increment_j),nVert_j);  % increment vertex
            iFixed = false;
        end
    else                             % it is turn for obstacle j to have fixed vertex while i marches
        if (IsVertexTangent([vx_j(jIndex),vy_j(jIndex)],obst_i,iIndex)==true)  
                % if so, then check if vertex on obstacle j is also tangent
            if IsVertexTangent([vx_i(iIndex),vy_i(iIndex)],obst_j,jIndex)==true
                BothTangent = true;   % since both are tangent, we're done
            else                      % need return to moving vertex on obstacle i 
                iFixed = true;
            end
        else                          % since vertex on obst j not tangent, iterate until it is
            iIndex = mymod((iIndex  + increment_i),nVert_i);  % increment vertex i
            iFixed = true;
        end
    end  % of if iTurn
end    % of while
itan_index = iIndex;  jtan_index = jIndex;
end    % of function MakeBiTangent