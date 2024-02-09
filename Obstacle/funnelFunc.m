function [funnelOps,pointOfInt,hoogPcells,nodeRespFun] = funnelFunc(nodes,HoogeveenPath,Max_Graph,Obstacle_List,boundingBox)
%UNTITLED3 Summary of this function goes here
pointOfInt = [];
Obstacle_coords = [];
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
%we build up the list of funnel openings as such:
%point1 x point 1 y point 2 x point 2 y
funnelOps = [];
r90 = [0 -1;
       1  0];
rm90 = [0 1;
       -1  0];

i = 2; 
hoogPcells = {};
curChain = [poses(1,:)];
nodeRespFun = [];
while i <= length(poses)
    used = false;
%     if curNode == 7
%         bo = 2+2
%     end
    cP1 = poses(i-1,:);
    cp1 = cP1;
    cP2 = poses(i,:);
    cp2 = cP2;
    slope = (cP2(2)-cP1(2))./(cP2(1)-cP1(1));
    if ~isinf(slope)
            yint = cp1(2)-cp1(1)*slope;
    else
            %actually xint
            yint = cp1(1);
    end
    [xout,yout] = linecirc(slope,yint,nodes(curNode,1),nodes(curNode,2),r);
    if xout(1) == xout(2) && yout(1) == yout(2)
        pi = [xout(1) yout(1)];
    elseif ~isnan(xout(1))
        if cP1(1)>cP2(1)
            is1x = xout(1)<=cP1(1) && xout(1)>=cP2(1);
        else
            is1x = xout(1)>=cP1(1) && xout(1)<=cP2(1);
        end

        if cP1(2)>cP2(2)
             is1y= yout(1)<=cP1(2) && yout(1)>=cP2(2);
        else
            is1y= yout(1)>=cP1(2) && yout(1)<=cP2(2);
        end
        if is1x && is1y
            pi = [xout(1) yout(1)];
        else
            pi = [xout(2) yout(2)];
        end
    end
    if curNode>1 & norm(cP1-nodes(curNode,:))>= r & norm(cP2-nodes(curNode,:))<= r
        %we have the entering of the node
        vec = nodes(curNode,:)-pi;
        %we 
        funnelOps(end+1,:) = [pi+(r90*vec')' pi+(rm90*vec')' curNode 0];
        used = true;
        pointOfInt(end+1,:) = pi;
        %we are ending the segment in between two circles
        curChain = [curChain;cp1;pi];
        hoogPcells{end+1} = curChain;
        nodeRespFun(end+1) = nodeOrd(nodeind); %we are in between two circs, no nodes
        curChain = [pi;cp2];
    end
    if curNode<length(nodes) & norm(cP1-nodes(curNode,:))<= r & norm(cP2-nodes(curNode,:))>= r
        %we have the exit of the node
        vec = nodes(curNode,:)-pi;
        funnelOps(end+1,:) = [pi+(r90*vec')' pi+(rm90*vec')' curNode 1];
        pointOfInt(end+1,:) = pi;
        used = true;
        nodeind = nodeind+1;
        curNode = nodeOrd(nodeind);
        curChain = [curChain;pi];
        hoogPcells{end+1} = curChain;
        nodeRespFun(end+1) = nodeOrd(nodeind-1);
        %now we restart the chain until we enter a new circ
        curChain = [pi];
    else
        curChain = [curChain;cp2];
        i = i+1;
    end
%I dont need verties now    
%     %check if we have any vertices
%     ind1 = find(Obstacle_coords(:,1) == cp1(1) & Obstacle_coords(:,2) == cp1(2));
%     ind2 = find(Obstacle_coords(:,1) == cp2(1) & Obstacle_coords(:,2) == cp2(2));
%     if ~isempty(ind1)
%         vec = r.*(cp1-cp2)./norm((cp1-cp2));
%         funnelOps(end+1,:) = [cp1+(r90*vec')' cp1+(rm90*vec')' -1 -1];
%         pointOfInt(end+1,:) = cp1;
%     end
%     if ~isempty(ind2)
%         vec = r.*(cp1-cp2)./norm((cp1-cp2));
%         funnelOps(end+1,:) = [cp2+(r90*vec')' cp2+(rm90*vec')' -1 -1];
%         pointOfInt(end+1,:) = cp2;
%     end
end
hoogPcells{end+1} = curChain;
nodeRespFun(end+1) = nodeOrd(nodeind);
funnelOps = [[nodes(1,:),nodes(1,:) 1 0];funnelOps;[nodes(end,:),nodes(end,:) nodeOrd(nodeind) 1]];
end