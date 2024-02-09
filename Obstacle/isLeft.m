%% Helper Function to determine if a Point C is "to the left" of a line AB
%  Inputs A,B,C, are 2x1 Cartesian coordinates of the three points
%
function [left] = isLeft(A,B,C)
det = (B(1)-A(1)) * (C(2)-A(2)) - (B(2)-A(2))*(C(1)-A(1));
if (det>0)
    left = true;
else
    left = false;
end  % of if/else
end  % of function