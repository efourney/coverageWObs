function [sensNodeLoc] = sensNodReduxOpt(srL,srT,r,targCent,x)
%find reduced set of sensing nodes
totCellofL = cell(length(targCent(:,1)),2);
curleng = 0;
for i = 2:length(x(:,1))
    lineSeg = x(i-1:i,:);
    for j = 1:length(targCent)
        [points] = pointsIntLineSegCirc(lineSeg(1,:),lineSeg(2,:),targCent(j,:),r);
        %so now we have set of key points/if theres any intersection
        if ~isempty(points)
            thetMat = thetCl{j};
            % chance for intersection!
            %check if any points are valid, ie, in the viewing region of
            %the target, not just the circle
            %gerate angles:
            vecCent2P = points-targCent(j,:);
            angs = atan2(vecCent2P(:,2),vecCent2P(:,1));
            val = zeros(1,length(angs));
            for k = 1:length(angs)
                curAng = angs(k);
                val(k) = any(angs(k)<=thetMat(:,2)+.001 & angs(k)+.001 >=thetMat(:,1));
            end
            points = points(val,:);
        end
    end
end
end