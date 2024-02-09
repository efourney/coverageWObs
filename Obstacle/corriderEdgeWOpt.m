function [points1,points2] = corriderEdgeWOpt(t,hoogP,funnelPs1,funnelPs2)
%t is triangulation of space%  
%hoog P is is the points making up the segment of the hoogeveen path
%between corrider opening
%funnelPs1 is 2x2 , each row is points that correspond with first point of hoogeveen path
%funnelPs2 is 2 points that correspond with last point of hoogeveen path

%find left and right sides
% hoogV1 = [hoogP(2,:)-hoogP(1,:) 0];
% hoogV2 = [hoogP(end,:)-hoogP(end-1,:) 0];
% fun1a = [funnelPs1(1,:)-hoogP(1,:),0];
% fun1b = [funnelPs1(2,:)-hoogP(1,:),0];
% fun2a = [funnelPs2(1,:)-hoogP(end,:),0];
% fun2b = [funnelPs2(2,:)-hoogP(end,:),0];
% v1a = cross(hoogV1,fun1a);
% v1b = cross(hoogV1,fun1b);
% v2a = cross(hoogV2,fun2a);
% v2b = cross(hoogV2,fun2b);
% 
% if sign(v1a(3)) == sign(v2a(3)) && sign(v1a(3)) ~=0 
%     edge1 = [funnelPs1(1,:);hoogP;funnelPs2(2,:)];
%     edge2 = [funnelPs1(2,:);hoogP;funnelPs2(2,:)];
% elseif sign(v1a(3)) ~=0 
%     edge1 = [funnelPs1(1,:);hoogP;funnelPs2(2,:)];
%     edge2 = [funnelPs1(2,:);hoogP;funnelPs2(1,:)];
% else %we are at the point, should only be on one side
%     if sign(v1b(3)) == sign(v2b(3))
%         edge1 = [funnelPs1(1,:);hoogP;funnelPs2(1,:)];
%         edge2 = [funnelPs1(2,:);hoogP;funnelPs2(2,:)];
%     else
%         edge1 = [funnelPs1(1,:);hoogP;funnelPs2(2,:)];
%         edge2 = [funnelPs1(2,:);hoogP;funnelPs2(1,:)];
%     end 
% end
fun1 = [];
fun2 = [];
%hoogP = hoogP([2 1],:);
if isCent(hoogP(1,:),hoogP(2,:),funnelPs1(1,:))
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [0 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [0 -1];
    else
        fun1 = [0 1];
    end
elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs1(1,:))
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [-1 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [-1 -1];
    else
        fun1 = [-1 1];
    end
else
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [1 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs1(2,:))
        fun1 = [1 -1];
    else
        fun1 = [1 1];
    end
end

if isCent(hoogP(1,:),hoogP(2,:),funnelPs2(1,:))
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [0 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [0 -1];
    else
        fun2 = [0 1];
    end
elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs2(1,:))
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [-1 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [-1 -1];
    else
        fun2 = [-1 1];
    end
else
    if isCent(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [1 0];
    elseif isLeft(hoogP(1,:),hoogP(2,:),funnelPs2(2,:))
        fun2 = [1 -1];
    else
        fun2 = [1 1];
    end
end

[~,ind1] = sort(fun1);
[~,ind2] = sort(fun2);

%hoogP = hoogP([2 1],:);
edge1 = [funnelPs1(ind1(1),:); hoogP;funnelPs2(ind2(1),:)];
edge2 = [funnelPs1(ind1(2),:); hoogP;funnelPs2(ind2(2),:)];

[points1] = findEdge(t,edge1);
[points2] = findEdge(t,edge2);
% points1 = edge1;
% points2 = edge2;
end

function [constraints]= findEdge(t,edge)
triEdg = [];
triInd = [];
for i = 1:length(t.ConnectivityList(:,1))
    curP = t.ConnectivityList(i,:);
    curP = t.Points(curP,:);
    triEdg = [triEdg;curP;curP(1,:);[NaN NaN]];
    triInd = [triInd,i i i i 0];
end
[xi,yi,indz] = polyxpoly(triEdg(:,1),triEdg(:,2),edge(:,1),edge(:,2));
indz = [indz(:,2) indz(:,1)];
%need to sort them in correct visitation order
for k = 1:indz(end,1)
    curInd = indz(:,1) == k;
    curPoin = [xi(curInd),yi(curInd)];
    distVec = vecnorm(curPoin-edge(k,:),2,2);
    [~,inds] = sort(distVec);
    indzTemp = indz(curInd,2);
    curPoin = curPoin(inds,:);
    xi(curInd) = curPoin(:,1);
    yi(curInd) = curPoin(:,2);
    indz(curInd,2) = indzTemp(inds);
end


%initialize with first pointd
%following code inspired by optimization 
constraints = [];
for i = 1:length(xi)
    segInd = indz(i,:);
    potPoins = triEdg(segInd(2):segInd(2)+1,:);
    check = false;
    if ~isempty(constraints)
        potDoubling = [constraints(end,1:2);constraints(end,3:4)];
        check = (all(potDoubling(1,:) == potPoins(1,:)) && all(potDoubling(2,:) == potPoins(2,:)))||...
            (all(potDoubling(2,:) == potPoins(1,:)) && all(potDoubling(1,:) == potPoins(2,:)));

    end
    if xi(i) == triEdg(segInd(2),1) && yi(i) == triEdg(segInd(2),2)
            %check if we are at a vertex-for now ignore MORE COMPLICATED IF we do
    %not know we can ignore verticies-we know we arent touching! need 2
    %adjust
    elseif ~check %ignore duplicates
        constraints = [constraints;[potPoins(1,:), potPoins(2,:)]];
    end
end
%logInd = abs(xi(1:end-1)-xi(2:end))<=.0000000001 & abs(yi(1:end-1)-yi(2:end))<=.0000000001;
% indsOfVer = find(logInd);
% trackWhichVer = diff(indsOfVer);
% indSwitch = find(trackWhichVer ~=1); %index when new vertex starts & ends
% i = 1;
% whichVerTrack = 0;
% while i < length(xi)
%     if any(i ==indsOfVer)
%         %we are at vertex, only issue if repeated which should be
%         whichVerTrack = whichVerTrack+1;
%         if i == 1
%             while i <= whichVerTrack(1)
%                 xi(i) == 0;
%                 yi(i) == 0;
%                 ii(i,:) == [0,0];
%             end
%         else
% 
%         end
%     end
%     i = i+1;
%end
end