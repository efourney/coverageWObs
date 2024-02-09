%% for clockwise (right) traversal around boundary of a polygon, is vertex "b"
%  a reflex vertex?  To answer that question, we preceeding vertex "a" and
%  following vertex "c".  Note that the inputs (a,b,c) are the indices of
%  the vertices.  We must find the 2D coordinates of the vertex
%
function [BoolOut] = isRightReflex(a_index,b_index,c_index)
global T                         % problem triangulation 
%
vA = T.Points(a_index,:); vB = T.Points(b_index,:); vC = T.Points(c_index,:);
det = (vB(1)-vA(1)) * (vC(2)-vB(2)) - (vC(1)-vB(1)) * (vB(2)-vA(2));
if (det <= 0)
    BoolOut = true;
else
    BoolOut = false;
end
end