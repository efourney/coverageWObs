%% Helper function "mymod" does modulo arithmetic on vertex integer intervals
%  The MATLAB "mod" function assumes that the integer 0 is in the range of
%  numbers for which mod applies.  But we index arrays starting with the
%  index "1" instead of zero.  This code accounts for that difference.
function [output] = mymod(input,nVert)
output = input;            % default 
if (input==0)
    output = nVert;
else
    if (input>nVert)
        output = mod(input,nVert);
    else
        output = input;
    end
end
end   % of mymod

