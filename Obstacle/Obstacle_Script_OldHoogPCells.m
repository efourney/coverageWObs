%% Script for testing code with obstacles.
    % for create and process obstacle-related data
Obstacle_Global                                    % read obstacle simulation parameters
PreProcessObstacleData();                          % set up data and check conflicts
r = SenseRadius;
% for i=1:(NumNodes-1)                               % temporary HACK until obstacle analysis is done
%    for j=i:NumNodes
%        SObstacle(i,j) = true;
%        SObstacle(j,i) = SObstacle(i,j);
%    end
% end
    % Booleans that indicate if an obstacle is active.  This is currently a
    % hack for the special case of a rectangle and triangular object.
% SObstacle(1,6) = false; SObstacle(6,1) = false;
% SObstacle(1,2) = false; SObstacle(2,1) = false;
% SObstacle(2,3) = false; SObstacle(3,2) = false;
% SObstacle(4,5) = false; SObstacle(5,4) = false;
% SObstacle(7,9) = false; SObstacle(9,7) = false;
% SObstacle(8,9) = false; SObstacle(9,8) = false;

Polys = polyshape({boundingBoxTrue(:,1),Obstacle_List(1).Vertices(:,1),Obstacle_List(2).Vertices(:,1),Obstacle_List(3).Vertices(:,1),Obstacle_List(4).Vertices(:,1)},...
    {boundingBoxTrue(:,2),Obstacle_List(1).Vertices(:,2),Obstacle_List(2).Vertices(:,2),Obstacle_List(3).Vertices(:,2),Obstacle_List(4).Vertices(:,2)});
%Polys = polyshape({[-5 -5 15 15],[5 6 7],[2 2 4 4 3]},{[-10 10 10 -10],[-2 -1 -2],[1 3 3 .7 2]});
plot(Polys)
T = triangulation(Polys);             % MATLAB triangulate function        
triplot(T);

%TargetNodes1 = TargetNodes(5,:)+[0 .5];
TargetNodes1 = TargetNodes([1 2 5 6],:);
TargetNodes1 = TargetNodes;
[vwCl,~,thetCl,oclRay] = genViewCellInfTri(TargetNodes1,Polys,Obstacle_List,SenseRadius);
[nodeLocs, baseList] = findBaseAndPlaceNodes(thetCl,oclRay,TargetNodes1,r,vwCl);
figure; hold on;
for i = 1:length(vwCl)
    if any(i==baseList)
        plot(vwCl(i),'EdgeColor','blue','LineWidth',3)
    else
        plot(vwCl(i),'EdgeColor','black','LineWidth',3)
    end
end
plot(nodeLocs(:,2),nodeLocs(:,3),'r*','LineWidth',2)
for i = 1:length(nodeLocs(:,1))
    nodLoc = nodeLocs(i,2:3);
    centLoc = TargetNodes(nodeLocs(i,1),:);
    plot([nodLoc(1) centLoc(1)],[nodLoc(2) centLoc(2)],'r--','LineWidth',1.5)
end
% plot(vwCl(7))
% plot(vwCl(8))
% plot(vwCl(9))
% plot(vwCl(10))
% plot(vwCl(11))
% plot(vwCl(12))
% plot(vwCl(13))
% plot(vwCl(14))
% plot(vwCl(15))
% plot(vwCl(16))
% plot(vwCl(17))
% plot(vwCl(18))
% plot(vwCl(19))
% plot(vwCl(20))
% plot(vwCl(21))
% plot(vwCl(22))
% plot(vwCl(23))
% plot(vwCl(24))
plot(Obstacle_List(1),'FaceColor','black')
plot(Obstacle_List(2),'FaceColor','black')
plot(Obstacle_List(3),'FaceColor','black')
plot(Obstacle_List(4),'FaceColor','black')
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    %viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  

TargetNodes1 = TargetNodes;
[~,inds] = sort(nodeLocs(:,1));
nodeLocs = nodeLocs(inds,:);
sensingNodes = [TargetNodes1(1,:);nodeLocs(:,2:3)];



    % construct maximally connected graph (including obstacles) 
%Max_Graph1 = MaxConnectedObstacleGraph(TargetNodes); 
visbGraph = emVisGraph(sensingNodes,Obstacle_List);
Max_Graph = emMaxFroVis(sensingNodes,visbGraph);
PlotMaxConnectedGraph(Max_Graph);
    % Construct first part of the overall solution from the Hoogeveen algorithm
HoogeveenPath = ObstacleHoogeveen(Max_Graph);
%get list of node locations
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



boundingBoxTrue = [-3,-3;-3,7;10,7;10,-3];
boundingBoxBig = [-30,-30;-30,70;100,70;100,-30];
%[funnelOps,pointOfInt,hoogPcells,nodeRespFun] = funnelFunc(TargetNodes,HoogeveenPath,Max_Graph,Obstacle_List,boundingBoxTrue);
%we now have a different way to place funnel openings
[funnelOps,pointOfInt,hoogPcells,nodeRespFun] = funnelFunc2(sensingNodes,TargetNodes,vwCl,HoogeveenPath,Max_Graph,Obstacle_List,boundingBoxTrue);

figure
Copy_of_PlotMaxConnectedGraph(Max_Graph);
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
end
% for i = 1:length(funnelOps)
%     plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'b')
% end

%check if any obstsacles are intersected and then if so trim/change the
%funnel Ops point
for i = 2:length(funnelOps(:,1))-1
    isCon = noInterferObsLinSeg(funnelOps(i,1:2),funnelOps(i,3:4),Obstacle_List);
    if ~isCon
       [pA,pB] = getLine4funnel(funnelOps(i,1:2),funnelOps(i,3:4),Obstacle_List,pointOfInt(i-1,:));
       funnelOps(i,1:4) = [pA,pB];
    end
end

%now we need to check fotr pairwise intersections
for i = 2:length(funnelOps(:,1))-2
    mid1 = pointOfInt(i-1,:);
    polyline1 = [funnelOps(i,1:2);funnelOps(i,3:4)];
    mid2 = pointOfInt(i,:);
    polyline2 = [funnelOps(i+1,1:2);funnelOps(i+1,3:4)];
    if ~all(mid1 == mid2)
        [xi,yi,~] = polyxpoly(polyline1(:,1),polyline1(:,2),polyline2(:,1),polyline2(:,2));
        if ~isempty(xi)
           %we have one intersection
           test1 = [xi yi;polyline1(1,:)];
           test2 = [xi yi;polyline1(2,:)];
           rise = polyline1(1,2)-polyline1(2,2);
           if rise ~=0
               ys1 = [yi polyline1(1,2) mid1(2)];
               ys2 = [yi polyline1(2,2) mid1(2)];
               [~,inds1] = sort(ys1);
               [~,inds2] = sort(ys2);
               if inds1(2) == 3
                   polyline1 = test1;
               elseif inds2(2) == 3
                   polyline1 = test2;
               end
           else
               xs1 = [xi polyline1(1,1) mid1(1)];
               xs2 = [xi polyline1(2,1) mid1(1)];
               [~,inds1] = sort(xs1);
               [~,inds2] = sort(xs2);
               if inds1(2) == 3
                   polyline1 = test1;
               elseif inds2(2) == 3
                   polyline1 = test2;
               end
           end
           %second side of the funnel
           test1 = [xi yi;polyline2(1,:)];
           test2 = [xi yi;polyline2(2,:)];
           rise = polyline2(1,2)-polyline2(2,2);
           if rise ~=0
               ys1 = [yi polyline2(1,2) mid2(2)];
               ys2 = [yi polyline2(2,2) mid2(2)];
               [~,inds1] = sort(ys1);
               [~,inds2] = sort(ys2);
               if inds1(2) == 3
                   polyline2 = test1;
               elseif inds2(2) == 3
                   polyline2 = test2;
               end
           else
               xs1 = [xi polyline2(1,1) mid2(1)];
               xs2 = [xi polyline2(2,1) mid2(1)];
               [~,inds1] = sort(xs1);
               [~,inds2] = sort(xs2);
               if inds1(2) == 3
                   polyline2 = test1;
               elseif inds2(2) == 3
                   polyline2 = test2;
               end
           end
        end
        funnelOps(i,1:4)=[polyline1(1,:),polyline1(2,:)];
        funnelOps(i+1,1:4)=[polyline2(1,:),polyline2(2,:)];
    end
end

figure
Copy_of_PlotMaxConnectedGraph(Max_Graph);
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
end
for i = 1:length(funnelOps)
    plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'b')
end

%[vwCl,vwClLine] = genViewCellInfTri(TargetNodes,Polys,Obstacle_List,SenseRadius);



%HANDLE SAME POINT/IN AND OUT HERE

% myObs = [Obstacle_List,polyshape(boundingBox(:,1),boundingBox(:,2))];
% verts = [];
% totalCon = [];
% for i = 1:length(myObs)
%     curInd = length(verts);
%     n4con = length(myObs(i).Vertices(:,1));
%     con = [(1:n4con)',[(2:n4con)';1]];
%     con = con+curInd;
%     totalCon = [totalCon;con];
%     verts = [verts;myObs(i).Vertices];
% end
% totalCon = totalCon+length(TargetNodes);
% verts = [TargetNodes;verts];
% myN = length(verts(:,1));
% myM = length(funnelOps(:,1));
% %verts = [verts;funnelOps(:,1:2);funnelOps(:,3:4)];
% %totalCon = [totalCon;[(myN+1:myN+myM)',(myN+myM+1:myN+2*myM)']];
% DT = delaunayTriangulation(verts(:,1),verts(:,2),totalCon);
% figure
% TF = isInterior(DT);
% triplot(DT.ConnectivityList(TF,:),DT.Points(:,1),DT.Points(:,2))  
% hold on
% for i = 1:length(poses)-1
%     plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
% end
% for i = 1:length(funnelOps)
%     plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'k')
% end
% Copy_of_PlotMaxConnectedGraph(Max_Graph);
% 





boundingBoxTrue = [-3,-3;-3,7;10,7;10,-3];
boundingBoxBig = [-30,-30;-30,70;100,70;100,-30];


Polys = polyshape({boundingBoxTrue(:,1),Obstacle_List(1).Vertices(:,1),Obstacle_List(2).Vertices(:,1),Obstacle_List(3).Vertices(:,1),Obstacle_List(4).Vertices(:,1)},...
    {boundingBoxTrue(:,2),Obstacle_List(1).Vertices(:,2),Obstacle_List(2).Vertices(:,2),Obstacle_List(3).Vertices(:,2),Obstacle_List(4).Vertices(:,2)});
%Polys = polyshape({[-5 -5 15 15],[5 6 7],[2 2 4 4 3]},{[-10 10 10 -10],[-2 -1 -2],[1 3 3 .7 2]});
plot(Polys)
T = triangulation(Polys);             % MATLAB triangulate function        
triplot(T);

TargetNodes1 = TargetNodes(3,:);
TargetNodes1 = TargetNodes;
[vwCl,vwClLine] = genViewCellInfTri(TargetNodes1,Polys,Obstacle_List,SenseRadius);
figure; hold on;
plot(vwCl(1))
plot(vwCl(2))
plot(vwCl(3))
plot(vwCl(4))
plot(vwCl(5))
plot(vwCl(6))
plot(vwCl(7))
plot(vwCl(8))
plot(vwCl(9))
plot(vwCl(10))
plot(Obstacle_List(1),'FaceColor','black')
plot(Obstacle_List(2),'FaceColor','black')
plot(Obstacle_List(3),'FaceColor','black')
plot(Obstacle_List(4),'FaceColor','black')
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  

figure
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
end
for i = 1:length(funnelOps)
    plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'b')
end
plot(vwCl(1))
plot(vwCl(2))
plot(vwCl(3))
plot(vwCl(4))
plot(vwCl(5))
plot(vwCl(6))
plot(vwCl(7))
plot(vwCl(8))
plot(vwCl(9))
plot(vwCl(10))
plot(Obstacle_List(1),'FaceColor','black')
plot(Obstacle_List(2),'FaceColor','black')
plot(Obstacle_List(3),'FaceColor','black')
plot(Obstacle_List(4),'FaceColor','black')




hourGlassList = Obstacle_List(1);
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
%for i = 2:length(funnelOps(:,1))-2
ACELL = {};
BCELL = {};
targView = {};
targ2view = [];

ACELL2 = {};
BCELL2 = {};
targView2 = {};
targ2view2 = [];

listOfLines = [];
for i = 1:length(funnelOps(:,1))-1
    funnelPs1 = [funnelOps(i,1:2);funnelOps(i,3:4)];
    funnelPs2 = [funnelOps(i+1,1:2);funnelOps(i+1,3:4)];
    %checking node responsibility
    if funnelOps(i,6) == 0 & funnelOps(i+1,6) == 1
        node2V = funnelOps(i,5);
        cell2v = vwCl(funnelOps(i,5));
        nodeloc2v = TargetNodes(node2V,:);
    else
        node2V = nan;
        cell2v = polyshape.empty;
        nodeloc2v = [nan nan];
    end
    
    [points1,points2,A_c,B_c,indsOfAOpt,myFun,pointscrossed] = corriderEdge(T,hoogPcells{i},funnelPs1,funnelPs2,Obstacle_List,nodeloc2v,cell2v,r);
    
    %now we check if we actually covered the point
    if ~isnan(node2V) && isempty(indsOfAOpt)
        %we need to visit the node
        %we want to "stop" the path at the node
        curPath = hoogPcells{i};
        curPathLong = curPath;
        %we know that the target is on the path since we can only enter if
        %statement when node2v isnt nan
        indOfTar = find(curPath(:,1) == TargetNodes(node2V,1) & curPath(:,2) == TargetNodes(node2V,2));
        curPath = curPath(1:indOfTar(1),:); %end at the node
        %now we want to try and see if we can just "grab" a line segment
        %from the viewing cell
        viewCellPolyV = [cell2v.Vertices;cell2v.Vertices(1,:)];
        [xi,yi,indz] = polyxpoly(viewCellPolyV(:,1),viewCellPolyV(:,2),curPath(:,1),curPath(:,2));
        indz = [indz(:,2) indz(:,1)];
        %need to sort them in correct visitation order
        for k = 1:indz(end,1)
            curInd = indz(:,1) == k;
            curPoin = [xi(curInd),yi(curInd)];
            distVec = vecnorm(curPoin-curPath(k,:),2,2);
            [~,inds] = sort(distVec);
            indzTemp = indz(curInd,2);
            curPoin = curPoin(inds,:);
            xi(curInd) = curPoin(:,1);
            yi(curInd) = curPoin(:,2);
            indz(curInd,2) = indzTemp(inds);
        end
        %now we can look for the "first" intersection;
        validLineSeg = [];
        k = 1;
        indOfViewFin = [];
        while k <=length(xi) && isempty(validLineSeg)
            check1 = norm(TargetNodes(node2V,:) - [xi(k) yi(k)])<=r;
            %hoog path must intersect where targ can be viewed
            %we want to check the the line segment of viewing cell is
            %INTERNAL aka one side must be in the circle
            indOfView = indz(k,2);
            linesegp1 = viewCellPolyV(indOfView,:);
            linesegp2 = viewCellPolyV(indOfView+1,:);
            check2 = norm(TargetNodes(node2V,:) - linesegp1)<=r ||...
                norm(TargetNodes(node2V,:) - linesegp2)<=r;
            if norm(linesegp1 - [xi(k) yi(k)])<=eps || norm(linesegp2 - [xi(k) yi(k)])<=eps 
                %we need to change check2 since we want to get some line
                %segment in the circle

                % MAYBE WE DONT WANT THIS CHECK

%                 check2 = norm(TargetNodes(node2V,:) - linesegp1)<=r &&...
%                 norm(TargetNodes(node2V,:) - linesegp2)<=r;

            end
            if check1 
                validLineSeg = [linesegp1;linesegp2];
                indOfViewFin = indz(k,1);
            end
            k = k+1;
        end
        %now, if we have a valid line segment, we can be done!
        if ~isempty(validLineSeg)
            validLineSeg4fun = pointsIntLineSegCirc(validLineSeg(1,:),validLineSeg(2,:),nodeloc2v,r);
            %it should be 2 points, we need to change the hoogeveen path
            %for now ERROR FIX
            validLineSeg4fun = [[xi(k) yi(k)];[xi(k) yi(k)]];

            curPath1 = [curPath(1:indOfViewFin,:); [xi(k) yi(k)]];
            %for number 2 we need the rest of the hoogevewen path
            curPath2 = [[xi(k) yi(k)];curPathLong(indOfViewFin+1:end,:)];

            %now we call function 2x to get the hourglasses, we add to
            %master lists
            [points11,points21,A_c1,B_c1,indsOfAOpt1,myFun1,pointscrossed1] = corriderEdge(T,curPath1,funnelPs1,...
                validLineSeg4fun,Obstacle_List,[nan nan],polyshape.empty,r);
            %indsOfAOpt1 = length(A_c1) we know this line segment will view
            %the target
            ACELL{end+1} = A_c1;
            BCELL{end+1} = B_c1;
            targView{end+1}=indsOfAOpt1;
            targ2view(end+1) = nan;
            [points12,points22,A_c2,B_c2,indsOfAOpt2,myFun2,pointscrossed2] = corriderEdge(T,curPath2,validLineSeg4fun,...
                funnelPs2,Obstacle_List,[nan nan],polyshape.empty,r);
            ACELL{end+1} = A_c2;
            BCELL{end+1} = B_c2;
            targView{end+1}=indsOfAOpt2;
            targ2view(end+1) = nan;
            myFun = [myFun1 myFun2];
            listOfLines = [listOfLines; pointscrossed1;pointscrossed2];
        else %we need to go to center will not happen rn 
%             [points1N,points2N,A_cN,B_cN,indsOfAOptN,myFunN,tNN] = corriderEdge(T,curPathLong,funnelPs1,...
%                 [TargetNodes(node2V,:);TargetNodes(node2V,:)],Obstacle_List,TargetNodes(node2V,:),cell2v,r);
%             %we need to find the first index of indsOfAOptN, and we know
%             %that the path should go there then "turn around"
%             try
%             indOfFirstView = indsOfAOptN(1); %will exist since the target is included
%             catch
%                 f = 4;
%             end
%             ACELL{end+1} = A_cN(1:indOfFirstView);
%             BCELL{end+1} = B_cN(1:indOfFirstView);
%             targView{end+1}=indOfFirstView;
%             %now we need to go "back" to the line before
%             lineSegA = A_cN{indOfFirstView-1};
%             lineSegB = B_cN{indOfFirstView-1};
%             if lineSegA(1,1) == 0
%                 %vertical line from
%                 funnelN = [lineSegB(end-1),-lineSegB(2);
%                     lineSegB(end-1),lineSegB(1)];
%             else
%                 %pull x range first
%                 x1 = -lineSegB(2);
%                 x2 = lineSegB(1);
%                 m = lineSegA(4,1);
%                 b = lineSegB(3);
%                 funnelN = [x1,m*x1+b;
%                     x2,m*x2+b];
%             end
%             %so now we have a new line to "head back from"
%             %we need to find the "last" time the hoogeveen path hits this
%             %line
%             [xi,yi,indz] = polyxpoly(funnelN(:,1),funnelN(:,2),curPathLong(:,1),curPathLong(:,2));
%             indz = [indz(:,2) indz(:,1)];
%             if isempty(indz)
%                 f = 4;
%             end
%             %need to sort them in correct visitation order
%             for k = 1:indz(end,1)
%                 curInd = indz(:,1) == k;
%                 curPoin = [xi(curInd),yi(curInd)];
%                 distVec = vecnorm(curPoin-curPathLong(k,:),2,2);
%                 [~,inds] = sort(distVec);
%                 indzTemp = indz(curInd,2);
%                 curPoin = curPoin(inds,:);
%                 xi(curInd) = curPoin(:,1);
%                 yi(curInd) = curPoin(:,2);
%                 indz(curInd,2) = indzTemp(inds);
%             end
%             %we can take the last one and the last section of hoogeveen
%             %path
%             curPath = [[xi(end) yi(end)];curPathLong(indz(end,1)+1:end-1,:);curPathLong(end,:)];
%             %now we can call the funnel func with no requirements
%             [points1N,points2N,A_cN,B_cN,indsOfAOptN,myFunN2,tNN] = corriderEdge(T,curPath,funnelN,...
%                 funnelPs2,Obstacle_List,[nan nan],polyshape.empty,r);
%             ACELL{end+1} = A_cN;
%             BCELL{end+1} = B_cN;
%             targView{end+1}=indsOfAOptN;
%             myFun = [myFunN,myFunN2];
        end
    else
        ACELL{end+1} = A_c;
        BCELL{end+1} = B_c;
        targView{end+1}=indsOfAOpt;
        targ2view(end+1) = node2V;
        listOfLines = [listOfLines; pointscrossed];
        hourGlassList = [hourGlassList,myFun];
    end
end


x = obstacleConOpt(ACELL,BCELL,targView,targ2view,TargetNodes,r);

figure
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
end
for i = 1:length(funnelOps)
    plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'b')
end
plot(vwCl(1))
plot(vwCl(2))
plot(vwCl(3))
plot(vwCl(4))
plot(vwCl(5))
plot(vwCl(6))
plot(vwCl(7))
plot(vwCl(8))
plot(vwCl(9))
plot(vwCl(10))
plot(Obstacle_List(1),'FaceColor','black')
plot(Obstacle_List(2),'FaceColor','black')
plot(Obstacle_List(3),'FaceColor','black')
plot(Obstacle_List(4),'FaceColor','black')
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
for i = 2:length(hourGlassList)
    plot(hourGlassList(i))
    tL = triangulation(hourGlassList(i));
    triplot(tL)
end

for i = 1:length(listOfLines)
    plot(listOfLines(i,[1 3]),listOfLines(i,[2 4]),'*-')
end

figure
hold on
plot(vwCl(1))
plot(vwCl(2))
plot(vwCl(3))
plot(vwCl(4))
plot(vwCl(5))
plot(vwCl(6))
plot(vwCl(7))
plot(vwCl(8))
plot(vwCl(9))
plot(vwCl(10))
plot(Obstacle_List(1),'FaceColor','black')
plot(Obstacle_List(2),'FaceColor','black')
plot(Obstacle_List(3),'FaceColor','black')
plot(Obstacle_List(4),'FaceColor','black')
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,num2str(iplot))
    hold on;
        % plot sensing range as circles
    viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
plot(x(1:74,1),x(1:74,2),'LineWidth',5)

A_vec = {};
for j = 1:length(poses)-1
    figure
    hold on
    PlotMaxConnectedGraph(Max_Graph);
    for i = 1:length(poses)-1
        plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',5)
    end
    point1 = poses(j,:);
    point2 = poses(j+1,:);
    box1 = genConvCellyfirst(point1,point2,Obstacle_List,boundingBoxTrue);
    box2 = genConvCellxfirst(point1,point2,Obstacle_List,boundingBoxTrue);
    if xor(responsible(j) == 0,responsible(j+1)==0)
        if responsible(j) == 0
            %this means point1 is corner point
            box3 = genConvObsCorn(point1,point2,Obstacle_List,boundingBoxTrue);
        else
            %point 2 is the corner point
            box3 = genConvObsCorn(point2,point1,Obstacle_List,boundingBoxTrue);
        end
    else
        box3 = box1;
    end

    [~,indy] = sort([area(box1) area(box2) area(box3)]);
    switch indy(end)
        case 1
            boxt = box1;
        case 2
            boxt = box2;
        case 3
            boxt = box3;
    end

    boxt = intersect(boxt,polyshape(boundingBoxTrue(:,1),boundingBoxTrue(:,2)));
    Xc = Polyhedron(boxt.Vertices);
    if isempty(Xc.A)
        Xc = struct();
        rise = point2(2)-point1(2);
        run = point2(1)-point1(1);
        if run == 0
            %x is fixed, y is in range
            Xc.A = [1 0;-1 0;0 1;0 -1];
            Xc.b = [point2(1);-point2(1);max([point2(2),point1(2)]);-min([point2(2),point1(2)])];
        elseif rise == 0
            %y is fixed, x is in range
            Xc.A = [1 0;-1 0;0 1;0 -1];
            Xc.b = [max([point2(1),point1(1)]);-min([point2(1),point1(1)]);point2(2);-point2(2)];
        else
            %bound x and keep y on the line
            m  = rise/run;
            b = point1(2) - m*point1(1);
            %y is fixed to line, x is in range
            Xc.A = [1 0;-1 0;-m 1;m -1];
            Xc.b = [max([point2(1),point1(1)]);-min([point2(1),point1(1)]);b;-b];
        end

    end
    A_vec{end+1} = [Xc.A,Xc.b];
%     if j >=10
%         plot(box1)
%         plot(box2)
%         plot(box3)
%         title('plotting 3')
%     end
    plot(boxt)
end

cvx_begin
    variable x(15,2)
    minimize(norm(x(2,:)-x(1,:))+norm(x(3,:)-x(2,:))+norm(x(4,:)-x(3,:))+norm(x(5,:)-x(4,:))+...
        norm(x(6,:)-x(5,:))+norm(x(7,:)-x(6,:))+norm(x(8,:)-x(7,:))+norm(x(9,:)-x(8,:))+...
        norm(x(10,:)-x(9,:))+norm(x(11,:)-x(10,:))+norm(x(12,:)-x(11,:))+norm(x(13,:)-x(12,:))+...
        norm(x(14,:)-x(13,:))+norm(x(15,:)-x(14,:)))
    subject to
        x(1,1) == poses(1,1)
        x(1,2) == poses(1,2)
        norm(x(2,:)-poses(2,:)) <= 0.76475
        norm(x(3,:)-poses(3,:)) <= 0.76475
        norm(x(4,:)-poses(4,:)) <= 0.76475
        x(5,:) == poses(5,:)
        norm(x(6,:)-poses(6,:)) <= 0.76475
        x(7,:) == poses(7,:)
        norm(x(8,:)-poses(8,:)) <= 0.76475
        norm(x(9,:)-poses(9,:)) <= 0.76475
        norm(x(10,:)-poses(10,:)) <= 0.76475
        x(11,:) == poses(11,:)
        x(12,:) == poses(12,:)
        norm(x(13,:)-poses(13,:)) <= 0.76475
        x(14,:) == poses(14,:)
        norm(x(15,:)-poses(15,:)) <= 0.76475
cvx_end

poses1 = [3,0;0,0;0,3;0,-3];

cvx_begin
    variable x(6,2)
    minimize(norm(x(2,:)-x(1,:))+norm(x(3,:)-x(2,:)))
    subject to
        %max(abs(4-4*x(1,1)),5-x(1,2))<=.5
        x(5,2) +  x(5,1) + x(6,1)<=1
        x(5,2) >=0
        x(5,1) >=0
        x(6,1) >=0
        norm(x(5,:)*x(1,1) + x(5,:)*x(2,1) + x(6,:)*x(3,1)-poses1(4,:))<=.76475
cvx_end


figure
hold on
    %PlotMaxConnectedGraph(Max_Graph);
    for i = 1:length(x)-1
        plot([x(i,1),x(i+1,1)],[x(i,2),x(i+1,2)],'r','LineWidth',5)
    end
    for i = 1:length(poses1)
        viscircles(poses1(i,:),0.76475)
    end

cvx_begin
    variable x(17,2)
    minimize(norm(x(2,:)-x(1,:))+norm(x(3,:)-x(2,:))+norm(x(4,:)-x(3,:))+norm(x(5,:)-x(4,:))+...
        norm(x(6,:)-x(5,:))+norm(x(7,:)-x(6,:))+norm(x(8,:)-x(7,:))+norm(x(9,:)-x(8,:))+...
        norm(x(10,:)-x(9,:))+norm(x(11,:)-x(10,:))+norm(x(12,:)-x(11,:))+norm(x(13,:)-x(12,:))+...
        norm(x(14,:)-x(13,:))+norm(x(15,:)-x(14,:))+norm(x(16,:)-x(15,:))+norm(x(17,:)-x(16,:)))
    subject to
        x(1,1) == poses(1,1)
        x(1,2) == poses(1,2)
        norm(x(2,:)-poses(2,:)) <= 0.76475
        norm(x(3,:)-poses(3,:)) <= 0.76475
        norm(x(17,:)-poses(17,:)) <= 0.76475
        norm(x(6,:)-poses(6,:)) <= 0.76475
        norm(x(8,:)-poses(8,:)) <= 0.76475
        norm(x(12,:)-poses(12,:)) <= 0.76475
        norm(x(10,:)-poses(10,:)) <= 0.76475
        norm(x(11,:)-poses(11,:)) <= 0.76475
        norm(x(15,:)-poses(15,:)) <= 0.76475
        A_vec{1}(:,1:2)*[x(2,1) x(2,2)]'<=A_vec{1}(:,3)
        A_vec{2}(:,1:2)*[x(2,1) x(2,2)]'<=A_vec{2}(:,3)
        A_vec{2}(:,1:2)*[x(3,1) x(3,2)]'<=A_vec{2}(:,3)
        A_vec{3}(:,1:2)*[x(3,1) x(3,2)]'<=A_vec{3}(:,3)
        A_vec{3}(:,1:2)*[x(4,1) x(4,2)]'<=A_vec{3}(:,3)
        A_vec{4}(:,1:2)*[x(4,1) x(4,2)]'<=A_vec{4}(:,3)
        A_vec{4}(:,1:2)*[x(5,1) x(5,2)]'<=A_vec{4}(:,3)
        A_vec{5}(:,1:2)*[x(5,1) x(5,2)]'<=A_vec{5}(:,3)
        A_vec{5}(:,1:2)*[x(6,1) x(6,2)]'<=A_vec{5}(:,3)
        A_vec{6}(:,1:2)*[x(6,1) x(6,2)]'<=A_vec{6}(:,3)
        A_vec{6}(:,1:2)*[x(7,1) x(7,2)]'<=A_vec{6}(:,3)
        A_vec{7}(:,1:2)*[x(7,1) x(7,2)]'<=A_vec{7}(:,3)
        A_vec{7}(:,1:2)*[x(8,1) x(8,2)]'<=A_vec{7}(:,3)
        A_vec{8}(:,1:2)*[x(8,1) x(8,2)]'<=A_vec{8}(:,3)
        A_vec{8}(:,1:2)*[x(9,1) x(9,2)]'<=A_vec{8}(:,3)
        A_vec{9}(:,1:2)*[x(9,1) x(9,2)]'<=A_vec{9}(:,3)
        A_vec{9}(:,1:2)*[x(10,1) x(10,2)]'<=A_vec{9}(:,3)
        A_vec{10}(:,1:2)*[x(10,1) x(10,2)]'<=A_vec{10}(:,3)
        A_vec{10}(:,1:2)*[x(11,1) x(11,2)]'<=A_vec{10}(:,3)
        A_vec{11}(:,1:2)*[x(11,1) x(11,2)]'<=A_vec{11}(:,3)
        A_vec{11}(:,1:2)*[x(12,1) x(12,2)]'<=A_vec{11}(:,3)
        A_vec{12}(:,1:2)*[x(12,1) x(12,2)]'<=A_vec{12}(:,3)
        A_vec{12}(:,1:2)*[x(13,1) x(13,2)]'<=A_vec{12}(:,3)
        A_vec{13}(:,1:2)*[x(13,1) x(13,2)]'<=A_vec{13}(:,3)
        A_vec{13}(:,1:2)*[x(14,1) x(14,2)]'<=A_vec{13}(:,3)
        A_vec{14}(:,1:2)*[x(14,1) x(14,2)]'<=A_vec{14}(:,3)
        A_vec{14}(:,1:2)*[x(15,1) x(15,2)]'<=A_vec{14}(:,3)
        A_vec{15}(:,1:2)*[x(15,1) x(15,2)]'<=A_vec{15}(:,3)
        A_vec{15}(:,1:2)*[x(16,1) x(16,2)]'<=A_vec{15}(:,3)
        A_vec{16}(:,1:2)*[x(16,1) x(16,2)]'<=A_vec{16}(:,3)
        A_vec{16}(:,1:2)*[x(17,1) x(17,2)]'<=A_vec{16}(:,3)
cvx_end

figure
hold on
    PlotMaxConnectedGraph(Max_Graph);
    for i = 1:length(x)-1
        plot([x(i,1),x(i+1,1)],[x(i,2),x(i+1,2)],'r','LineWidth',5)
    end
% 
% point1 = poses(1,:);
% point2 = poses(2,:);
% box1 = genConvCellyfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% box2 = genConvCellxfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% if area(box1)>=area(box2)
%      boxt = box1;
% else
%     boxt = box2;
% end
% boxt = intersect(boxt,polyshape(boundingBoxTrue(:,1),boundingBoxTrue(:,2)));
% plot(boxt)
% 
% point1 = poses(2,:);
% point2 = poses(3,:);
% box1 = genConvCellyfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% box2 = genConvCellxfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% if area(box1)>=area(box2)
%      boxt = box1;
% else
%     boxt = box2;
% end
% boxt = intersect(boxt,polyshape(boundingBoxTrue(:,1),boundingBoxTrue(:,2)));
% plot(boxt)
% 
% 
% point1 = poses(3,:);
% point2 = poses(4,:);
% box1 = genConvCellyfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% box2 = genConvCellxfirst(point1,point2,Obstacle_List,boundingBoxTrue);
% if area(box1)>=area(box2)
%      boxt = box1;
% else
%     boxt = box2;
% end
% boxt = intersect(boxt,polyshape(boundingBoxTrue(:,1),boundingBoxTrue(:,2)));
% plot(boxt)


%     % Prepare for the continuous sensor location optimization
% FixedNodeIncidence = GraphToIncidence(HoogeveenPath);  % convert graph path to Incidence vector 
%     % set up the initial guess of the sensing node locations needed by the optimizer
%     %  Note that the optimizer works with a "flat" vector representation of the
%     %  sensing nodes: S = (S_1x, S_1y, S_2x, S_2y, ...., S_nx, S_ny);
% SenseNodes0 = zeros(1,((NumNodes-1) * ProblemD));         % allocate space for the sensing node vector, in incidence form
% for i=1:(NumNodes-1)
%     index = (i-1)*ProblemD + 1;
%     SenseNodes0(index:(index+ProblemD-1)) = TargetNodes((i+1),:);
% end
% % %% optimize the sensing node locations, given fixed order of tour through nodes
% % OptimizedSenseNodes = ObstaclePathOptimization(SenseNodes0);
% %     % now plot the results
% PlotObstacleProblemData();
% hold on;
% PlotObstacleSolutionGraph(OptimizedSenseNodes);
