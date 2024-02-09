function [points1,points2,A_vec,b_vec,indOfT,myFun,points2cross] = corriderEdge(t,hoogP,funnelPs1,funnelPs2,Obstacle_List,targetLoc,vwCell,r)
global Obstacle_List
indOfT = [];
A_vec = {};
b_vec = {};
tN = [];
points2cross = [];

if isnan(targetLoc(1))
    coveredT = true;
    indOfT = 0;
else
    coveredT = false;
end

%An Efficient Algorithm for Euclidean Shortest Paths Among Polygonal Obstacles in the Plane
%S. Kapoor,1 S. N. Maheshwari,1 and J. S. B. Mitchell2


%I assumed hoogP was starting and stopping at intersection w/ funnel1 and
%2, need to make adjustment now
% [xiS,yiS,indS] = polyxpoly(hoogP(:,1),hoogP(:,2),funnelPs1(:,1),funnelPs1(:,2));
% [xiT,yiT,indT] = polyxpoly(hoogP(:,1),hoogP(:,2),funnelPs2(:,1),funnelPs2(:,2));
% %first hoogP point to be included should be indz(1,1)+1
% hoogP = [xiS(1), yiS(2); hoogP(indS(1,1)+1:indT(1,1),:);xiT(1),yiT(1)];

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

if isCent(hoogP(end-1,:),hoogP(end,:),funnelPs2(1,:))
    if isCent(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
        fun2 = [0 0];
    elseif isLeft(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
        fun2 = [0 -1];
    else
        fun2 = [0 1];
    end
elseif isLeft(hoogP(end-1,:),hoogP(end,:),funnelPs2(1,:))
    if isCent(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
        fun2 = [-1 0];
    elseif isLeft(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
        fun2 = [-1 -1];
    else
        fun2 = [-1 1];
    end
else
    if isCent(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
        fun2 = [1 0];
    elseif isLeft(hoogP(end-1,:),hoogP(end,:),funnelPs2(2,:))
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

poly1 = polyshape([funnelPs1(1,1) funnelPs1(2,1) funnelPs2(1,1) funnelPs2(2,1)],...
    [funnelPs1(1,2) funnelPs1(2,2) funnelPs2(1,2) funnelPs2(2,2)]);
poly2 = polyshape([funnelPs1(1,1) funnelPs1(2,1) funnelPs2(2,1) funnelPs2(1,1)],...
    [funnelPs1(1,2) funnelPs1(2,2) funnelPs2(2,2) funnelPs2(1,2)]);

if area(poly1) > area(poly2)
    edge1 = [funnelPs1(1,:); hoogP;funnelPs2(2,:)];
    edge2 = [funnelPs1(2,:); hoogP;funnelPs2(1,:)];
else
    edge1 = [funnelPs1(1,:); hoogP;funnelPs2(1,:)];
    edge2 = [funnelPs1(2,:); hoogP;funnelPs2(2,:)];
end


[points1] = findEdge(t,edge1);
[points2] = findEdge(t,edge2);
myFun = polyshape([points1(:,1); points2(end:-1:1,1)],[points1(:,2); points2(end:-1:1,2)]);
%return;
%temporary
%check if our viewing cell is 
% if ~coveredT
%     potArea = intersect(myFun,vwCell);
%     if potArea.NumRegions ==0
%         %we need to go to the target and back
% 
%     end


% 
% end

if myFun.NumRegions == 0
    return; %we are done, its line over line
end


tN1 = triangulation(myFun);

tN.Points = tN1.Points;
tN.ConnectivityList = tN1.ConnectivityList;
%we need all the points on the left and right side added
points2add = [];
for i = 1:length(points1)
    if ~any(tN1.Points(:,1) == points1(i,1) & tN1.Points(:,2) == points1(i,2))
        points2add(end+1,:) = points1(i,:);
    end
end
for i = 1:length(points2)
    if ~any(tN1.Points(:,1) == points2(i,1) & tN1.Points(:,2) == points2(i,2))
        points2add(end+1,:) = points2(i,:);
    end
end

tN.Points = unique([tN1.Points;points2add],'rows','stable');
tN.ConnectivityList = tN1.ConnectivityList;

% figure;
% hold on;
% plot(myFun)
% triplot(tN);
% plot(funnelPs1(:,1),funnelPs1(:,2));
% plot(funnelPs2(:,1),funnelPs2(:,2))
% plot(Obstacle_List(1))
% plot(Obstacle_List(2))

% points1 = edge1;
% points2 = edge2;

%new way
%we know left and right sides
lineSeg = [];

indsOfLeft = [];
for i = 1:length(points1)
    pointinq = points1(i,:);
    indInTri = find(tN.Points(:,1) == pointinq(1) & tN.Points(:,2) == pointinq(2));
    try
    indsOfLeft(end+1) = indInTri;
    catch
        d = 5;
    end
end

indsOfRight = [];
for i = 1:length(points2)
    pointinq = points2(i,:);
    indInTri = find(tN.Points(:,1) == pointinq(1) & tN.Points(:,2) == pointinq(2));
    indsOfRight(end+1) = indInTri;
end

[C,indOLP,indORP] = intersect(points1,points2,'rows');
k = 1;
while k <= length(C(:,1))
    check1 = C(k,1) == funnelPs1(1,1) && C(k,2) == funnelPs1(1,2);
    check2 = C(k,1) == funnelPs1(2,1) && C(k,2) == funnelPs1(2,2);
    check3 = C(k,1) == funnelPs2(2,1) && C(k,2) == funnelPs2(2,2);
    check4 = C(k,1) == funnelPs2(1,1) && C(k,2) == funnelPs2(1,2);
    if (check1 & check2) | (check3 & check4)
        k = k+1;
    elseif check1 | check2 | check3 | check4
        C(k,:) = [];
    else
        k = k+1;
    end
end

indsOfPinch = [];
for i = 1:length(C(:,1))
    pointinq = C(i,:);
    indInTri = find(tN.Points(:,1) == pointinq(1) & tN.Points(:,2) == pointinq(2));
    indsOfPinch(end+1) = indInTri;
end

%we have the ith element of indsOfLeft is the index of the ith point of the left side in the
%triangulation for the shape
%L then R
points2cross = [funnelPs1(1,:) funnelPs1(2,:)];
pinchPinOrder = [];
for i = 1:length(indsOfLeft)
    firstInd = indsOfLeft(i);
    %check if we are at a pinch point
    if any(indsOfPinch == firstInd)
        %we know this point must be included and we can move on
        pinchPinOrder(end+1) = firstInd;
        points2cross = [points2cross;[tN.Points(firstInd,:) tN.Points(firstInd,:)]];
    else
        %find all triangles associated with this point
        logMat1 = tN.ConnectivityList == firstInd;
        triInQ = tN.ConnectivityList(any(logMat1,2),:);
        triInQ = triInQ(:);
        %remove all the points that are either pinch points OR the first in
        pInQ = unique(setdiff(triInQ,[indsOfPinch,firstInd]));
        %now we have a list of indicies on the right side, we need to sort
        %them in order that they appear in the right side
%         [~,indOrd4curPL] = sort(indOrdRight(pInQ));
%         pInQ = pInQ(indOrd4curPL);
        %was trying to be smart but i should just do this:
        pInQF = [];
        for p = 1:length(indsOfRight)
            if any(pInQ == indsOfRight(p))
                pInQF(end+1) = indsOfRight(p);
            end
        end
        points2cross = [points2cross;[tN.Points(firstInd*ones(length(pInQF),1),:), tN.Points(pInQF,:)]];
    end
end
points2cross = [points2cross;funnelPs2(1,:) funnelPs2(2,:)];
points2cross = unique(points2cross,'rows','stable');

%if there are any pinch points, we check if the path *has* to observe the
%target from them
if ~isempty(pinchPinOrder) & ~coveredT
    locOfP = tN.Points(pinchPinOrder,:);
    for i = 1:length(pinchPinOrder)
        if norm(locOfP(i,:)-targetLoc)<=r & inpolygon(locOfP(i,1),locOfP(i,2),vwCell.Vertices(:,1),vwCell.Vertices(:,2))
            coveredT = true;
            indOfT = find(points2cross(:,1) == locOfP(i,1) & points2cross(:,2) == locOfP(i,2));
        end
    end
    if ~coveredT
        for i = 1:length(pinchPinOrder)-1
            lineSegP = [locOfP(i,:); locOfP(i+1,:)];
            [xi,yi] = polyxpoly(lineSegP(:,1),lineSegP(:,2),[vwCell.Vertices(:,1);vwCell.Vertices(1,1)],[vwCell.Vertices(:,2);vwCell.Vertices(1,2)]);
            for j = 1:length(xi)
                if norm([xi(j) yi(j)]-targetLoc)<=r & inpolygon(xi(j), yi(j),vwCell.Vertices(:,1),vwCell.Vertices(:,2))
                    coveredT = true;
                    indOfT = find(points2cross(:,1) == locOfP(i+1,1) & points2cross(:,2) == locOfP(i+1,2));
                end
            end
        end
    end
end

%then we have handled the target being viewed?





%OLD WAY FOR VISITATION ORDER
% %get order of visitation from funnel 1 to 2
% triEdg = [];
% triInd = [];
% for i = 1:length(tN.ConnectivityList(:,1))
%     curP = tN.ConnectivityList(i,:);
%     curP = tN.Points(curP,:);
%     triEdg = [triEdg;curP;curP(1,:);[NaN NaN]];
%     triInd = [triInd,i i i i 0];
% end
% [xi,yi,indz] = polyxpoly(triEdg(:,1),triEdg(:,2),hoogP(:,1),hoogP(:,2));
% indz = [indz(:,2) indz(:,1)];
% %need to sort them in correct visitation order
% for k = 1:indz(end,1)
%     curInd = indz(:,1) == k;
%     curPoin = [xi(curInd),yi(curInd)];
%     distVec = vecnorm(curPoin-hoogP(k,:),2,2);
%     [~,inds] = sort(distVec);
%     indzTemp = indz(curInd,2);
%     curPoin = curPoin(inds,:);
%     xi(curInd) = curPoin(:,1);
%     yi(curInd) = curPoin(:,2);
%     indz(curInd,2) = indzTemp(inds);
% end
% 
% %we now have a list of the points that must be visited
% [pinchP, ind1,ind2] = intersect(points1,points2,'rows','stable');
% %now we have the pinch points, we want to enumerate all the point pairs
% %that represent the edges 
% 
% lineSeg = [funnelPs1(1,:),funnelPs1(2,:)];
% points4bigM = hoogP(1,:);
% for i = 1:length(xi)
%     segInd = indz(i,:);
%     potPoins = triEdg(segInd(2):segInd(2)+1,:);
%     if all(lineSeg(end,1:2) == potPoins(1,:)) &&  all(lineSeg(end,3:4) == potPoins(2,:))
%     elseif all(lineSeg(end,1:2) == potPoins(2,:)) &&  all(lineSeg(end,3:4) == potPoins(1,:))
%     else
%         doNormProt = true;
%         %check that we have not identified a pinch point makes more elegant
%         %but it is not necesary if optimal solition is truly found
% %         poinaP = (potPoins(1,1)==pinchP(:,1) & potPoins(1,2)==pinchP(:,2));
% %         poinbP = (potPoins(2,1)==pinchP(:,1) & potPoins(2,2)==pinchP(:,2));
% %         if any(poinaP) && any(poinbP)
% %             %check which should be first
% %             indexOfPinch = sort(find(poinaP),find(poinbP));
% %             lineSeg([end+1 end+2],:) = [pinchP(indexOfPinch,:),pinchP(indexOfPinch,:)];
% %             doNormProt = false;
% %         elseif any(poinaP)
% %             %we need to check if we are 
% %         elseif any(poinbP)
% %         end
% 
%         %check that we have one edge from each side, this eliminates
%         %intersections with verticies
%         if doNormProt
%         point1L = any(points1(:,1) == potPoins(1,1) & points1(:,2) == potPoins(1,2));
%         point1R = any(points2(:,1) == potPoins(1,1) & points2(:,2) == potPoins(1,2));
%         point2L = any(points1(:,1) == potPoins(2,1) & points1(:,2) == potPoins(2,2));
%         point2R = any(points2(:,1) == potPoins(2,1) & points2(:,2) == potPoins(2,2));
%         if (point1L && point2R) || (point1R && point2L)
%             lineSeg(end+1,:) = [potPoins(1,:),potPoins(2,:)];
%             points4bigM(end+1,:) = [xi(i) yi(i)];
%         end
%         end
%     end
% end
% potPoins = funnelPs2;
% if all(lineSeg(end,1:2) == potPoins(1,:)) &&  all(lineSeg(end,3:4) == potPoins(2,:))
% elseif all(lineSeg(end,1:2) == potPoins(2,:)) &&  all(lineSeg(end,3:4) == potPoins(1,:))
% else
%     lineSeg(end+1,:) = [funnelPs2(1,:),funnelPs2(2,:)];
%     points4bigM(end+1,:) = hoogP(end,:);
% end



%now we need to generate A matricies IN ORDER;
lineSeg = points2cross;
A_vec = {};
b_vec = {};
for i = 1:length(lineSeg(:,1))
    a = lineSeg(i,1:2);
    b = lineSeg(i,3:4);
    sRis = b(2)-a(2);
    sRun = b(1)-a(1);
    if all(a==b)
        A_vec{end+1} = [0 1;0 -1;1 0;-1 0];
        b_vec{end+1} = [a(2);-a(2);a(1);-a(1)];
    elseif sRun == 0
        A_vec{end+1} = [0 1;0 -1;1 0;-1 0];
        b_vec{end+1} = [max([b(2),a(2)]);-min([b(2),a(2)]);b(1);-b(1)];
    else
         m  = sRis/sRun;
         bee = a(2) - m*a(1);
        %y is fixed to line, x is in range
         A_vec{end+1} = [1 0;-1 0;-m 1;m -1];
         b_vec{end+1} = [max([b(1),a(1)]);-min([b(1),a(1)]);bee;-bee];
    end
    
    if ~coveredT & i<length(lineSeg(:,1))
        %check if point is between this line seg and next one
        %will be covered by last one dont need to worry
        points4hull =  unique([a;b;lineSeg(i+1,1:2);lineSeg(i+1,3:4)],'rows');
        isTri = false;
        %if it is a triangle, we proceed, if it is a line, we would have
        %already checked
        if length(points4hull) == 3
            viewTri = intersect(vwCell,polyshape(points4hull(:,1),points4hull(:,2)));
            isTri = true;
        end

      
        if isTri && viewTri.NumRegions>=1
%             %we need to check that this section of the hoogeveen path could
%             %view the target
%             %we need to check out line segment if it can view the target
%             path = points4bigM(i:i+1,:);
%             [xP,yP,~] = polyxpoly(path(:,1),path(:,2),vwCell.Vertices(:,1),vwCell.Vertices(:,2));
%             points2check = [path;[xP,yP]];
%             [points] = pointsIntLineSegCirc(path(1,:),path(2,:),targetLoc,r);
%             points2check = [points2check;points];
%             %add points from new helper
%             tri_con = Polyhedron(viewTri.Vertices);
%             isBigM = false;
%             for l = 1:length(points2check)
%                 curP = points2check(l,:);
%                 if norm(curP-targetLoc)<=r && inpolygon(curP(1),curP(2),vwCell.Vertices(:,1),vwCell.Vertices(:,2))
%                     isBigM = true;
%                 end
%             end
%             %pot error-did I incorrectly assume line seg connects points?
%             %finally, if triangle contains the target it's obviously on
%             %path, only time the path would bend in a traingle
%             if inpolygon(targetLoc(1),targetLoc(2),x_tri(k),y_tri(k))
%                 isBigM = true;
%             end
            %we need to check if the hoogeveen path views the target from
            %the cell-need 2 adjust
            viewingSubSec = regions(viewTri);
            for p = 1:length(viewingSubSec)
                isBigM = true;
                if isBigM
                    tri_con = Polyhedron(viewingSubSec(p).Vertices);
                    A_vec{end+1} = tri_con.A;
                    b_vec{end+1} = tri_con.b;
                    indOfT = [indOfT,length(A_vec)];
                end
            end
        end
    end
end

end

function [points]= findEdge(t,edge)
global Obstacle_List
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
if isempty(indz)
    %we did not leave a traingle, hence a straight line works
    points = [edge(1,:);edge(end,:)];
else
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

%we need to construct dual graph
trianglesIncluded = unique(triInd(indz(:,2)));
A = zeros(length(trianglesIncluded));
NAMES = {};
for i = 1:length(trianglesIncluded)
    NAMES{end+1} = num2str(trianglesIncluded(i));
    temp = intersect(neighbors(t, trianglesIncluded(i)),trianglesIncluded);
    for j = 1:length(temp)
        indInMat = find(temp(j) == trianglesIncluded);
        A(i,indInMat)  = 1;
    end
end
G = graph(A,NAMES,'omitselfloops');
PATH = shortestpath(G,num2str(triInd(indz(1,2))),num2str(triInd(indz(end,2))));
%create new polygon opstacles that are just the bounding box minus the
%triangles in the path

%do not even need to do the trimming from the duel ?
% all triangles not in trianglesIncluded are added as obstacles, use 
T = t;
triObs = 1:length(t.ConnectivityList);
indicies =  ones(1,length(t.ConnectivityList));
indicies(trianglesIncluded) = 0;
triObs = triObs(logical(indicies));
if isempty(Obstacle_List)
    %should never have called this, just connect the points 
    return;
end
allObs = Obstacle_List(1);
for i = 2:length(Obstacle_List)
    allObs = union(allObs,Obstacle_List(i));
end
for i = 1:length(triObs)
    connectInd = T.ConnectivityList(triObs(i),:);
    allObs = union(allObs,polyshape(t.Points(connectInd,1),T.Points(connectInd,2)));
end
%check for any corner-to corner connections and add box
myMat = allObs.Vertices;
indexOfRep = [];
for  i = 1:length(myMat)
    curPoin = myMat(i,:);
    if ~isnan(curPoin(1)) && any(curPoin(1)==myMat([1:i-1 i+1:end],1) & curPoin(2)==myMat([1:i-1 i+1:end],2))
        indexOfRep(end+1) = i;
    end
end
% A = myMat(:,1);
% [~, uniqueIdx] = unique(A);
% dupeIdx1 = ismember( A, A( setdiff( 1:numel(A), uniqueIdx ) ) );
% A = myMat(:,2);
% [~, uniqueIdx] = unique(A);
% dupeIdx2 = ismember( A, A( setdiff( 1:numel(A), uniqueIdx ) ) );
% indexOfRep = find(dupeIdx1 & dupeIdx2);
polys2add = polyshape.empty;
myepsy = .05;
for i = 1:length(indexOfRep)
    curPoin = myMat(indexOfRep(i),:);
    if ~isnan(curPoin(1))
        polys2add(end+1) = polyshape(curPoin(1)+[-myepsy -myepsy myepsy myepsy],...
            curPoin(2)+[-myepsy myepsy myepsy -myepsy]);
    end
end
for i = 1:length(polys2add)
    allObs = union(allObs,polys2add(i));
end

%we need to figure out 
%now we have "bubbles" around all the corners so we should have no issues
%with crossing a point connection
%we need to handle a hole
% if allObs.NumHoles>0 %should be 1
%     holeShape = holes(allObs);
%     convexHoleHull = convhull(holeShape);
%     %now we know where we can cheat with a little triangle
%     innerVert = convexHoleHull.Vertices;
%     outerVert = boundingBox;
%     myDistVec = vecnorm(innerVert-outerVert(1,:),2,2);
% 
% 
% end

% if allObs.NumRegions>1
% realObsList = polyshape.empty;
% curVer = [];
% for i = 1:length(allObs.Vertices)
%     if ~isnan(allObs.Vertices(i,1)) 
%         curVer(end+1,:) = allObs.Vertices(i,:);
%     else
%         realObsList(end+1) = polyshape(curVer(:,1),curVer(:,2));
%         curVer = [];
%     end
% end
% realObsList(end+1) = polyshape(curVer(:,1),curVer(:,2));
% else
%     
% end
realObsList = allObs;

visbGraph = emVisGraph(edge([1 end],:),realObsList);
Max_Graph = emMaxFroVis(edge([1 end],:),visbGraph);
pointsCell = Max_Graph.Edges.Paths{1,1};
points = [pointsCell{1},pointsCell{2}];
end
%initialize with first pointd
%following code inspired by
%https://medium.com/@reza.teshnizi/the-funnel-algorithm-explained-visually-41e374172d2d
% left = edge(1,:);
% right = edge(1,:);
% final = [];
% for i = 1:length(xi)
%     segInd = indz(i,:);
%     potPoins = triEdg(segInd(2):segInd(2)+1,:);
%     if isempty(final)
%         A = edge(segInd(1),:);
%         B = edge(segInd(1)+1,:);
%     else
%         A = final(end,:);
%         B = [xi(i),yi(i)];
%     end
% %     A = edge(segInd(1),:);
% %     B = edge(segInd(1)+1,:);
%     if isLeft(A,B,potPoins(1,:))
%         leftp = potPoins(1,:);
%         rightp = potPoins(2,:);
%     elseif isRight(A,B,potPoins(1,:))
%         leftp = potPoins(2,:);
%         rightp = potPoins(1,:);
%     else
%         if isLeft(A,B,potPoins(2,:))
%             leftp = potPoins(2,:);
%             rightp = potPoins(1,:);
%         elseif isRight(A,B,potPoins(2,:))
%             leftp = potPoins(1,:);
%             rightp = potPoins(2,:);
%         else %wrong and bad
%             leftp = potPoins(1,:);
%             rightp = potPoins(2,:);
%         end
%     end
%     skip = false;
%     skip = all(rightp == right(end,:)) && all(leftp == left(end,:));
%     if length(left(:,1)) == 1 && ~all(leftp == left(end,:))
%         left = [left;leftp];
%         if length(right(:,1)) == 1 && ~all(rightp == right(end,:))
%             right = [right;rightp];
%         end
%         skip = true;
%     elseif length(right(:,1)) == 1  && ~all(rightp == right(end,:))
%         right = [right;rightp];
%         skip = true;
%     elseif ~(skip) %we need to check for crossovers/create the tail
%         %one side
%         checking4tail = zeros(length(left(:,1)),1);
%         for j = 1:length(left(:,1))-1
%             checkcur = isLeft(left(j,:),left(j+1,:),rightp);
%             checking4tail(j) = double(checkcur);
%         end
%         inds2worry = find(checking4tail);
%         if ~isempty(inds2worry)
%             final = [final; left(1:inds2worry(end)+1,:)];
%             if ~all(left(inds2worry(end)+1,:)==leftp)
%                 left = [left(1:inds2worry(end)+1,:)];
%             else
%                 left = [left(1:inds2worry(end)+1,:)];
%             end
%             right = [left(inds2worry(end)+1,:);rightp];
%             right = [left(1:inds2worry(end)+1,:)];
%             left = final(end,:);
%             right = left;
%             skip = true;
%         end
%         %now other
%         if ~skip
%             checking4tail = zeros(length(right(:,1)),1);
%             for j = 1:length(right(:,1))-1
%                 checkcur = isRight(right(j,:),right(j+1,:),leftp);
%                 checking4tail(j) = double(checkcur);
%             end
%             inds2worry = find(checking4tail);
%             if ~isempty(inds2worry)
%                 final = [final; right(1:inds2worry(end)+1,:)];
%                 left = [right(inds2worry(end),:);leftp];
%                 left = right(1:inds2worry(end)+1,:);
%                 if ~all(right(inds2worry(end)+1,:)==rightp)
%                     right = [right(inds2worry(end)+1,:);rightp];
%                 else
%                     right = [right(inds2worry(end),:);rightp];
%                 end
%                 left = final(end,:);
%                 right = left;
%                 skip = true;
%             end
%         end
%     end
% 
%     %check if we have one same between them
%     if skip
%     elseif all(leftp == left(end,:)) && all(rightp == right(end,:))
%     elseif all(leftp == left(end,:))
%         %check if we need to replace or add to the right side
%         better = isLeft(right(end-1,:),right(end,:),rightp);
%         if better %replace
%             right(end,:) = rightp;
%         else %add
%             right(end+1,:) = rightp;
%         end
%     elseif all(rightp == right(end,:))
%         better = isRight(left(end-1,:),left(end,:),leftp);
%         if better %replace
%             left(end,:) = leftp;
%         else %add
%             left(end+1,:) = leftp;
%         end
%     else %we need to check against other two sides, and/or add both
%         %this should be impossible?
%     end
%     
% end
% points = final;
% L = left;
% R = right;
% figure;
% hold on;
% plot(left(:,1),left(:,2));
% plot(right(:,1),right(:,2));
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
% end