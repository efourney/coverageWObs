function [funnelOps,pointOfInt,hoogPcells,nodeRespFun] = funnelFunc2(nodes,targets,vwCl,HoogeveenPath,Max_Graph,Obstacle_List,boundingBox)
%UNTITLED3 Summary of this function goes here
pointOfInt = [];
Obstacle_coords = [];
hoogPcells = [];
nodeRespFun = [];
global SenseRadius
for i = 1:length(Obstacle_List)
    curver = Obstacle_List(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end

r = SenseRadius;
%check for contact with obstacle
curInd1 = 1;
poses = [];
responsible = [];
while curInd1<height(HoogeveenPath.Nodes)
    indEdge = find(HoogeveenPath.Edges.EndNodes(:,1) == curInd1);
    curInd2 = HoogeveenPath.Edges.EndNodes(indEdge,2);
    indsPath = find((Max_Graph.Edges.EndNodes(:,1) == curInd1 | Max_Graph.Edges.EndNodes(:,2) == curInd1) &...
        (Max_Graph.Edges.EndNodes(:,1) == curInd2 | Max_Graph.Edges.EndNodes(:,2) == curInd2));
    edgeOfInt = Max_Graph.Edges(indsPath,:);
    posesTemp = [];
    if edgeOfInt.EndNodes(1) ~= curInd1
        posesTemp(:,1) = edgeOfInt.Paths{1}{1}(end:-1:1)';
        posesTemp(:,2) = edgeOfInt.Paths{1}{2}(end:-1:1)';
    else
        posesTemp(:,1) = edgeOfInt.Paths{1}{1}';
        posesTemp(:,2) = edgeOfInt.Paths{1}{2}';
    end
    responsible(end+1) = curInd1;
    hTemp = size(posesTemp,1);
    responsible(end+1:end+hTemp-2) = zeros(1,hTemp-2);
    responsible(end+1) = curInd2;
    poses = cat(1,poses,posesTemp);
    curInd1 = curInd2;
end
inds2del = find(responsible ~=0 & responsible == [-1 responsible(1:end-1)]);
poses(inds2del,:) = [];
responsible(inds2del) = [];

nodeOrd = responsible(responsible~=0);
curNode = 1;
nodeind  = 1;
funnelOps = [];
curChain = [];
for i = 1:length(responsible)
    curChain = [curChain;poses(i,:)];
    if responsible(i) ~=0
        %this is a node
        if responsible(i) == 1
            %we just need a point for the funnel
            funnelOps(end+1,:) = [poses(i,:),poses(i,:)];
            pointOfInt(end+1,:) = poses(i,:);
            hoogPcells{end+1} = [curChain; poses(i,:)];
            curChain = poses(i,:);
        else
            curTarg=responsible(i);
            curTL = targets(curTarg,:);
            curNL = poses(i,:);
            lengthV = norm(curTL-curNL);
            viewClVert = [vwCl(curTarg).Vertices(:,:);vwCl(curTarg).Vertices(1,:)];
            if lengthV>0
                pFar1 = curNL + 2.15*r.*((curTL-curNL)./lengthV);
                pFar2 = curNL - 2.15*r.*((curTL-curNL)./lengthV);
                [xi1,yi1] = polyxpoly(viewClVert(:,1),viewClVert(:,2),[pFar1(1),curNL(1)],[pFar1(2),curNL(2)]);
                [xi2,yi2] = polyxpoly(viewClVert(:,1),viewClVert(:,2),[pFar2(1),curNL(1)],[pFar2(2),curNL(2)]);
                if isempty(xi1)
                    p1 = curNL;
                else
                    vecDiff1 = vecnorm([xi1,yi1]-curNL,2,2);
                    [~,inds1] = sort(vecDiff1);
                    p1 = [xi1(inds1(1)) yi1(inds1(1))];
                end
                if isempty(xi2)
                    p2 = curNL;
                else
                    vecDiff2 = vecnorm([xi2,yi2]-curNL,2,2);
                    [~,inds2] = sort(vecDiff2);
                    p2 = [xi2(inds2(1)) yi2(inds2(1))];
                end
                vec1 = [p1 p2];
            else
                vec1 = [curNL curNL];
            end
            %now we need to try another vector, this time "splitting" the
            %hoogeveen path angles
            %find incoming and outgoing paths:
            backPath = false;
            backP = [];
            forPath = false;
            forP = [];
            curInd = i-1;
            while ~backPath
                if curInd < 1
                    backPath = true;
                    backP = curNL;
                else
                    vecDif =  norm(curNL-poses(curInd,:));
                    if abs(vecDif)>eps
                        backPath = true;
                        backP = poses(curInd,:);
                    else
                        curInd = curInd-1;
                    end
                end
            end
            curInd = i+1;
            while ~forPath
                if curInd > length(responsible)
                    forPath = true;
                    forP = curNL;
                else
                    vecDif =  norm(curNL-poses(curInd,:));
                    if abs(vecDif)>eps
                        forPath = true;
                        forP = poses(curInd,:);
                    else
                        curInd = curInd+1;
                    end
                end
            end
            %so now we have points before and after, only befor eor only
            %after, or none
            vecOfAng = [];
            forPVec = forP-curNL;
            backPVec = backP-curNL;
            if norm(forPVec)>eps
                vecOfAng(end+1) = atan2(forPVec(2),forPVec(1));
            end
            if norm(backPVec)>eps
                vecOfAng(end+1) = atan2(backPVec(2),backPVec(1));
            end
            %we can just use angles average since we go back ways
            ang2use = mean(vecOfAng);
            if ~isnan(ang2use)
                %vec1 = [curNL+2.15*r.*[cos(ang2use) sin(ang2use)] curNL-2.15*r.*[cos(ang2use) sin(ang2use)]];
                pFar1 = curNL + 2.15*r.*[cos(ang2use) sin(ang2use)];
                pFar2 = curNL - 2.15*r.*[cos(ang2use) sin(ang2use)];
                [xi1,yi1] = polyxpoly(viewClVert(:,1),viewClVert(:,2),[pFar1(1),curNL(1)],[pFar1(2),curNL(2)]);
                [xi2,yi2] = polyxpoly(viewClVert(:,1),viewClVert(:,2),[pFar2(1),curNL(1)],[pFar2(2),curNL(2)]);
                if isempty(xi1)
                    p1 = curNL;
                else
                    vecDiff1 = vecnorm([xi1,yi1]-curNL,2,2);
                    [~,inds] = sort(vecDiff1);
                    p1 = [xi1(inds(1)) yi1(inds(1))];
                end
                if isempty(xi2)
                    p2 = curNL;
                else
                    vecDiff2 = vecnorm([xi2,yi2]-curNL,2,2);
                    [~,inds] = sort(vecDiff2);
                    p2 = [xi2(inds(1)) yi2(inds(1))];
                end
                vec2 = [p1 p2];
            else
                vec2 = [curNL curNL];
            end
            %so now we have two vectors 
            if norm(vec1(1:2)-vec1(3:4))>norm(vec2(1:2)-vec2(3:4))
                vecT = vec1;
            else
                vecT = vec2;
            end
            %now we have the vector funnel we need, it is fully in the sensing
            %viewing cell
            funnelOps(end+1,:) = vecT;
            pointOfInt(end+1,:) = curNL;
            hoogPcells{end+1} = [curChain; curNL];
            curChain = curNL;
        end
    end
end
% %we build up the list of funnel openings as such:
% %point1 x point 1 y point 2 x point 2 y
% funnelOps = [];
% r90 = [0 -1;
%        1  0];
% rm90 = [0 1;
%        -1  0];
% 
% i = 2; 
% hoogPcells = {};
% curChain = [poses(1,:)];
% nodeRespFun = [];
% while i <= length(poses)
%     used = false;
% %     if curNode == 7
% %         bo = 2+2
% %     end
%     cP1 = poses(i-1,:);
%     cp1 = cP1;
%     cP2 = poses(i,:);
%     cp2 = cP2;
%     slope = (cP2(2)-cP1(2))./(cP2(1)-cP1(1));
%     if ~isinf(slope)
%             yint = cp1(2)-cp1(1)*slope;
%     else
%             %actually xint
%             yint = cp1(1);
%     end
%     [xout,yout] = linecirc(slope,yint,nodes(curNode,1),nodes(curNode,2),r);
%     if xout(1) == xout(2) && yout(1) == yout(2)
%         pi = [xout(1) yout(1)];
%     elseif ~isnan(xout(1))
%         if cP1(1)>cP2(1)
%             is1x = xout(1)<=cP1(1) && xout(1)>=cP2(1);
%         else
%             is1x = xout(1)>=cP1(1) && xout(1)<=cP2(1);
%         end
% 
%         if cP1(2)>cP2(2)
%              is1y= yout(1)<=cP1(2) && yout(1)>=cP2(2);
%         else
%             is1y= yout(1)>=cP1(2) && yout(1)<=cP2(2);
%         end
%         if is1x && is1y
%             pi = [xout(1) yout(1)];
%         else
%             pi = [xout(2) yout(2)];
%         end
%     end
%     if curNode>1 & norm(cP1-nodes(curNode,:))>= r & norm(cP2-nodes(curNode,:))<= r
%         %we have the entering of the node
%         vec = nodes(curNode,:)-pi;
%         %we 
%         funnelOps(end+1,:) = [pi+(r90*vec')' pi+(rm90*vec')' curNode 0];
%         used = true;
%         pointOfInt(end+1,:) = pi;
%         %we are ending the segment in between two circles
%         curChain = [curChain;cp1;pi];
%         hoogPcells{end+1} = curChain;
%         nodeRespFun(end+1) = nodeOrd(nodeind); %we are in between two circs, no nodes
%         curChain = [pi;cp2];
%     end
%     if curNode<length(nodes) & norm(cP1-nodes(curNode,:))<= r & norm(cP2-nodes(curNode,:))>= r
%         %we have the exit of the node
%         vec = nodes(curNode,:)-pi;
%         funnelOps(end+1,:) = [pi+(r90*vec')' pi+(rm90*vec')' curNode 1];
%         pointOfInt(end+1,:) = pi;
%         used = true;
%         nodeind = nodeind+1;
%         curNode = nodeOrd(nodeind);
%         curChain = [curChain;pi];
%         hoogPcells{end+1} = curChain;
%         nodeRespFun(end+1) = nodeOrd(nodeind-1);
%         %now we restart the chain until we enter a new circ
%         curChain = [pi];
%     else
%         curChain = [curChain;cp2];
%         i = i+1;
%     end
% %I dont need verties now    
% %     %check if we have any vertices
% %     ind1 = find(Obstacle_coords(:,1) == cp1(1) & Obstacle_coords(:,2) == cp1(2));
% %     ind2 = find(Obstacle_coords(:,1) == cp2(1) & Obstacle_coords(:,2) == cp2(2));
% %     if ~isempty(ind1)
% %         vec = r.*(cp1-cp2)./norm((cp1-cp2));
% %         funnelOps(end+1,:) = [cp1+(r90*vec')' cp1+(rm90*vec')' -1 -1];
% %         pointOfInt(end+1,:) = cp1;
% %     end
% %     if ~isempty(ind2)
% %         vec = r.*(cp1-cp2)./norm((cp1-cp2));
% %         funnelOps(end+1,:) = [cp2+(r90*vec')' cp2+(rm90*vec')' -1 -1];
% %         pointOfInt(end+1,:) = cp2;
% %     end
% end
% hoogPcells{end+1} = curChain;
% nodeRespFun(end+1) = nodeOrd(nodeind);
% funnelOps = [[nodes(1,:),nodes(1,:) 1 0];funnelOps;[nodes(end,:),nodes(end,:) nodeOrd(nodeind) 1]];
end