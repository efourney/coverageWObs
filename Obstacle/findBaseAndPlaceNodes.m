function [nodeLocs, baseList] = findBaseAndPlaceNodes(thetCl,oclRay,TargetNodes,r,vwCl)
%first list uncovered nodes
uncovered = 2:size(TargetNodes,1);
uncovered = uncovered(randperm(length(uncovered)));
% uncovered = [5 4 3 2 6];
uncovered = [6 5 2 3 4 7];
nodeLocs = [];
baseList = [];
while ~isempty(uncovered)
    newbase = uncovered(1);
    baseList = [baseList, newbase];
    nodeLocs = [nodeLocs; [newbase, TargetNodes(newbase,:)]];
    %line segs
    baseLin = oclRay{newbase};
    baseThet = thetCl{newbase};
    newBaseNode = TargetNodes(newbase,:);
    %check for overlaps
    indsOfNewCover = [1];
    for i = 2:length(uncovered)
        overlapInd = uncovered(i);
        potNeighNode = TargetNodes(overlapInd,:);
        if norm(newBaseNode-potNeighNode)<=2*r %only check when theres potential
            confirmedNeigh = false;
            potNeighLin = oclRay{overlapInd};
            potNeighTh = thetCl{overlapInd};
            [xout,yout] = circcirc(newBaseNode(1),newBaseNode(2),r,potNeighNode(1),potNeighNode(2),r); 
            %convert to angles for both circles
            thetBaseQ = atan2(yout-newBaseNode(2),xout-newBaseNode(1));
            thetNeighQ = atan2(yout-potNeighNode(2),xout-potNeighNode(1));
%             if any(thetBaseQ == pi | thetBaseQ == -pi)
%                 thetBaseQ = [thetBaseQ,pi,-pi];
%             end
%             if any(thetNeighQ == pi | thetBaseQ == -pi)
%                 thetNeighQ = [thetNeighQ,pi,-pi];
%             end
            %we have covered our bases over the duality of options
            inRB = thetBaseQ > 2*pi; %easy way to get log 0s
            inRN = thetNeighQ > 2*pi;
            for j = 1:length(baseThet(:,1))
                curLow = baseThet(j,1);
                curHi = baseThet(j,2);
                inRB = curLow<=thetBaseQ+eps & thetBaseQ<=curHi+eps | inRB;
            end
            for j = 1:length(potNeighTh(:,1))
                curLow = potNeighTh(j,1);
                curHi = potNeighTh(j,2);
                inRN = curLow<=thetNeighQ+eps & thetNeighQ<=curHi+eps | inRN;
            end
            indOfOvr = find(inRN & inRB);
            if ~isempty(indOfOvr)
                angOfCross = thetBaseQ(indOfOvr(1));
                pointsOfInt = newBaseNode+r.*[cos(angOfCross) sin(angOfCross)];
                nodeLocs = [nodeLocs;[overlapInd, pointsOfInt]];
                indsOfNewCover = [indsOfNewCover, i];
                confirmedNeigh = true;
            end

            %now that we have checked for circular overlaps, we may need to
            %check if any line segments cross circular segments and cive
            %versa, but only if we have not identified the cross
            if ~confirmedNeigh %check for overlaps between base line segments and neighbor circle
                for k = 1:length(baseLin(:,1))
                    cp1 = baseLin(k,1:2);
                    cp2 = baseLin(k,3:4);
                    points2check = pointsIntLineSegCirc(cp1,cp2,potNeighNode,r);
                    if ~isempty(points2check) & ~confirmedNeigh
                        thetNeighQ = atan2(points2check(:,2)-potNeighNode(2),points2check(:,1)-potNeighNode(1));
                        inRN = thetNeighQ > 2*pi;
                        for j = 1:length(potNeighTh(:,1))
                            curLow = potNeighTh(j,1);
                            curHi = potNeighTh(j,2);
                            inRN = curLow<=thetNeighQ+eps & thetNeighQ<=curHi+eps | inRN;
                        end
                        if any(inRN)
                            indOfRN = find(inRN);
                            indOfRN = indOfRN(1);
                            pointsOfInt = points2check(indOfRN,:);
                            nodeLocs = [nodeLocs;[overlapInd, pointsOfInt]];
                            indsOfNewCover = [indsOfNewCover, i];
                            confirmedNeigh = true;
                        end
                    end
                end
            end

            if ~confirmedNeigh %check for overlaps between neigh line segments and base circle
                for k = 1:length(potNeighLin(:,1))
                    cp1 = potNeighLin(k,1:2);
                    cp2 = potNeighLin(k,3:4);
                    points2check = pointsIntLineSegCircNOINTERNALINCLUSION(cp1,cp2,newBaseNode,r);
                    if ~isempty(points2check) & ~confirmedNeigh
                        thetBaseQ = atan2(points2check(:,2)-newBaseNode(2),points2check(:,1)-newBaseNode(1));
                        inRB = thetBaseQ > 2*pi; %easy way to get log 0s
                        for j = 1:length(baseThet(:,1))
                            curLow = baseThet(j,1);
                            curHi = baseThet(j,2);
                            inRB = curLow<=thetBaseQ+eps & thetBaseQ<=curHi+eps | inRB;
                        end
                        if any(inRB)
                            angOfCross = thetBaseQ(inRB);
                            angOfCross = angOfCross(1);
                            pointsOfInt = newBaseNode+r.*[cos(angOfCross) sin(angOfCross)];
                            nodeLocs = [nodeLocs;[overlapInd, pointsOfInt]];
                            indsOfNewCover = [indsOfNewCover, i];
                            confirmedNeigh = true;
                        end
                    end
                end
            end
            %finally just check for line segment intersections, worse
            %possible complexity implemented here
            if ~confirmedNeigh %check for overlaps between neigh line segments and base circle
                for k = 1:length(potNeighLin(:,1))
                    for j = 1:length(baseLin(:,1))
                        if ~confirmedNeigh
                            %if they cross, at the point and we are done
                            [xi,yi] = linexline(potNeighLin(k,[1 3]), potNeighLin(k,[2 4]),...
                                baseLin(j,[1 3]), baseLin(j,[2 4]), false);
                            if ~isnan(xi)
                                nodeLocs = [nodeLocs;[overlapInd, [xi yi]]];
                                indsOfNewCover = [indsOfNewCover, i];
                                confirmedNeigh = true;
                            end
                        end
                    end
                end
            end
            %finally, check if we have a neighbor INSIDE the viewing cell
            %but with no overlap
            if ~confirmedNeigh && norm(potNeighNode-newBaseNode)<=r
                if isinterior(vwCl(newbase),potNeighNode(1),potNeighNode(2)) %we know neighboir is in base vwcll
                    %check the angle of the point
                    thetBaseQ = atan2(potNeighNode(2)-newBaseNode(2),potNeighNode(1)-newBaseNode(1));
                    for j = 1:length(baseThet(:,1))
                        curLow = baseThet(j,1);
                        curHi = baseThet(j,2);
                        inRB = curLow<=thetBaseQ+eps & thetBaseQ<=curHi+eps | inRB;
                    end
                    %if it is in a section where the angle is right, done
                    if inRB
                        nodeLocs = [nodeLocs;[overlapInd, newBaseNode+r.*[cos(thetBaseQ) sin(thetBaseQ)]]];
                        indsOfNewCover = [indsOfNewCover, i];
                        confirmedNeigh = true;
                    end
                    %if not, we need to see if we can just use the base
                    if ~confirmedNeigh & length(baseLin) ~= 0
                        nodeLocs = [nodeLocs;[overlapInd, newBaseNode]];
                        indsOfNewCover = [indsOfNewCover, i];
                        confirmedNeigh = true;
                    elseif ~confirmedNeigh
                        %we need to use a line to intersect the polyshape,
                        %no worries about it beinf ourside circle
                        %point on circle 
                        boundaryP = newBaseNode+r.*[cos(thetBaseQ) sin(thetBaseQ)];
                        fullvwcellvert = [vwCl{newbase}.Vertices(:,:);vwCl{newbase}.Vertices(1,:)];
                        [XI,YI] = polyxpoly([newBaseNode(1) boundaryP(1)],[newBaseNode(2) boundaryP(2)],...
                            fullvwcellvert(:,1),fullvwcellvert(:,2));
                        normsort = vecnorm([[XI,YI]-newBaseNode],2,2);
                        [~,ind] = sort(normsort);
                        nodeLocs = [nodeLocs;[overlapInd, [XI(ind(1)),YI(ind(1))]]];
                        indsOfNewCover = [indsOfNewCover, i];
                        confirmedNeigh = true;
                    end

                elseif isinterior(vwCl{overlapInd},newBaseNode(1),newBaseNode(2)) %just use base node
                    nodeLocs = [nodeLocs;[overlapInd, newBaseNode]];
                    indsOfNewCover = [indsOfNewCover, i];
                    confirmedNeigh = true;
                end
            end
        end
    end
    if length(indsOfNewCover)>=2
        %this base has neighbors, and we need to worry about node placement
        if length(baseLin) == 0
            %there are no occlusion rays, hence the base node needs to be
            %on the boundary to hold bound
            nodeLocs(nodeLocs(:,1)==newbase,2:3) = newBaseNode+[cos(baseThet(1,1)) sin(baseThet(1,1))].*r;
        end
    end
    uncovered(indsOfNewCover) = [];
end
end