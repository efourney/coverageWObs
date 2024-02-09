%% Helper function.  It determines if the vertex list passed to it is a right-handed (clockwise) 
%  path on the boundary of a polygon (which is always described by counter-clockwise indexing
%  NOTE that this function ASSUMES that polygon vertices are in counter-clockwise order 
%  when polygons are created
function [right] = isRightWrap(vlist)
    % look at first two elements of the list/vector
v1 = vlist(1);  v2 = vlist(2); vdiff = abs(v1-v2);
    % now check on wrapping order based on index number.  If polygons are constructed with 
    % counter-clockwise order, then the vertex indices should be decreasing when traversing 
    % the boundary in clockwise order
if (v2<v1)                     % if true, this might be a clockwise traversal--but need to refine analysis
    if (vdiff==1)              % if this is true, then not a polygon wrap-around index change
        right = true;          % traversal must be clockwise
    else
        right = false;         % this case must be a wrap-around vertex index reseton left side
    end
else                           % case of index(v2) > index(v1)
    if (vdiff==1)              % if this is true, then not a polygon wrap-around index change
        right = false;         % traversal must be clockwise on lrft cirridor side
    else
        right = true;          % this case must be a wrap-around vertex index reset on right side
    end
end
end    % function
        

