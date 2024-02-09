%% Script for testing code with obstacles.
    % for create and process obstacle-related data


%requires linexline file from amtlab file exchange:
    %* Author: Preetham Manjunatha                                          *%
%* Github link: https://github.com/preethamam                           *%
%* Date: 02/08/2022                                                     *%
%************************************************************************%
%
%************************************************************************%
%
% Usage: [xi,yi] = linexline(L1x, L1y, L2x, L2y)




Obstacle_Global                                    % read obstacle simulation parameters
PreProcessObstacleData();                          % set up data and check conflicts
r = SenseRadius;

%build cell full obstacle vertices
cellOfVertsx = {boundingBoxTrue(:,1)};
cellOfVertsy = {boundingBoxTrue(:,2)};
for i = 1:length(Obstacle_List)
    cellOfVertsx{end+1} = Obstacle_List(i).Vertices(:,1);
    cellOfVertsy{end+1} = Obstacle_List(i).Vertices(:,2);
end
Polys = polyshape(cellOfVertsx,cellOfVertsy);
T = triangulation(Polys);             % MATLAB triangulate function        


TargetNodes1 = TargetNodes;
[vwCl,~,thetCl,oclRay] = genViewCellInfTri(TargetNodes1,Polys,Obstacle_List,SenseRadius);
[nodeLocs, baseList] = findBaseAndPlaceNodes(thetCl,oclRay,TargetNodes1,r,vwCl);

[~,inds] = sort(nodeLocs(:,1));
nodeLocs = nodeLocs(inds,:);
sensingNodes = [TargetNodes1(1,:);nodeLocs(:,2:3)];



    % construct maximally connected graph (including obstacles) 
%Max_Graph1 = MaxConnectedObstacleGraph(TargetNodes); 
visbGraph = emVisGraph(sensingNodes,Obstacle_List);
Max_Graph = emMaxFroVis(sensingNodes,visbGraph);

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


%[funnelOps,pointOfInt,hoogPcells,nodeRespFun] = funnelFunc(TargetNodes,HoogeveenPath,Max_Graph,Obstacle_List,boundingBoxTrue);
%we now have a different way to place funnel openings
[funnelOps,pointOfInt,hoogPcells,~] = funnelFunc2(poses,TargetNodes,vwCl,HoogeveenPath,Max_Graph,Obstacle_List,boundingBoxTrue);

%check if any obstsacles are intersected and then if so trim/change the
%funnel Ops point
%this should be unnecesary now because the lines are fully inside viewing
%regions, helps because corners are weird numerically
for i = 2:length(funnelOps(:,1))
    isCon = noInterferObsLinSeg(funnelOps(i,1:2),funnelOps(i,3:4),Obstacle_List);
    if ~isCon
       [pA,pB] = getLine4funnel(funnelOps(i,1:2),funnelOps(i,3:4),Obstacle_List,pointOfInt(i,:));
       funnelOps(i,1:4) = [pA,pB];
    end
end

%check if any funnel opening hits an obstacle vertex-nothing wrong
%theoretically but weird numeric behavoir we try to avoid by simply backing
%off vertex. 
% *Matlab polyshape complication, create list of all verticies*
obstacle_verticies = [];
for i = 1:length(ObstacleVertex_List)
    curver = [ObstacleVertex_List{1,i}{1,1} ObstacleVertex_List{1,i}{1,2}];
    obstacle_verticies = [obstacle_verticies;curver];
end

%numerically, weird behavoir from "touching" obstacles, this prevents
%weirdness
for i = 1:length(funnelOps(:,1))
   vert1 = funnelOps(i,1:2);
   vert2 = funnelOps(i,3:4);
   vec122 = vert2-vert1;
   if ~all(vert1 == vert2)
        vert1 = vert2 - vec122*.99;
        vert2 = vert1 + vec122*.99;
        funnelOps(i,:) = [vert1 vert2];
   end
end


%now we need to check fotr pairwise intersections
for i = 2:length(funnelOps(:,1))
    mid1 = pointOfInt(i-1,:);
    polyline1 = [funnelOps(i-1,1:2);funnelOps(i-1,3:4)];
    mid2 = pointOfInt(i,:);
    polyline2 = [funnelOps(i,1:2);funnelOps(i,3:4)];
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
        funnelOps(i-1,1:4)=[polyline1(1,:),polyline1(2,:)];
        funnelOps(i,1:4)=[polyline2(1,:),polyline2(2,:)];
    end
end

hourGlassList = Obstacle_List(1);
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
    %if funnelOps(i,6) == 0 & funnelOps(i+1,6) == 1
    if 2==3
        node2V = funnelOps(i,5);
        cell2v = vwCl(funnelOps(i,5));
        nodeloc2v = TargetNodes(node2V,:);
    else
        node2V = nan;
        cell2v = polyshape.empty;
        nodeloc2v = [nan nan];
    end
    
    [points1,points2,A_c,B_c,indsOfAOpt,myFun,pointscrossed] = corriderEdge(T,hoogPcells{i+1},funnelPs1,funnelPs2,Obstacle_List,nodeloc2v,cell2v,r);
    
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
hourGlassList(1) = []; %get rid of obstsacle

x = obstacleConOpt(ACELL,BCELL,targView,targ2view,TargetNodes,r);

[sensNodeLoc] = sensNodReduxOptSIMPL(vwCl,r,TargetNodes,x);

figure
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',2.5)
    if responsible(i) ~=0
    text(poses(i,1) + .1,poses(i,2)+.1,['S_' num2str(responsible(i))])
    end
end
text(poses(length(poses),1) + .1,poses(length(poses),2)+.1,['S_' num2str(length(find(responsible~=0)))])
for i = 1:length(funnelOps)
    plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'g')
end
for i = 1:length(vwCl)
    if any(i == baseList)
        plot(vwCl(i),'EdgeColor','b','FaceAlpha',0,'LineWidth',2)
    else
        plot(vwCl(i),'EdgeAlpha',0)
    end
end
for i = 1:length(Obstacle_List)
    plot(Obstacle_List(i),'FaceColor','black')
end
for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,['T_' num2str(iplot)])
    hold on;
        % plot sensing range as circles
    %viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
% for i = 2:length(hourGlassList)
%     plot(hourGlassList(i))
%     tL = triangulation(hourGlassList(i));
%     triplot(tL)
% end

% for i = 1:length(listOfLines)
%     plot(listOfLines(i,[1 3]),listOfLines(i,[2 4]),'*-')
% end
plot(x(:,1),x(:,2),'k','LineWidth',2.5)


figure
hold on
for i = 1:length(poses)-1
    plot([poses(i,1),poses(i+1,1)],[poses(i,2),poses(i+1,2)],'r','LineWidth',2.5)
    if responsible(i) ~=0
    text(poses(i,1) + .1,poses(i,2)+.1,['S_' num2str(responsible(i))])
    end
end
text(poses(length(poses),1) + .1,poses(length(poses),2)+.1,['S_' num2str(length(find(responsible~=0)))])
for i = 1:length(hourGlassList)
    plot(hourGlassList(i),'FaceColor',[0.4660 0.6740 0.1880])
end
for i = 1:length(funnelOps)
    plot(funnelOps(i,[1,3]),funnelOps(i,[2,4]),'color',[0.4660 0.6740 0.1880],'LineWidth',2)
end

for i = 1:length(Obstacle_List)
    plot(Obstacle_List(i),'FaceColor','black')
end

for iplot=1:length(TargetNodes)
    CurNode=[TargetNodes(iplot,1), TargetNodes(iplot,2)];
        % plot target nodes as "dots"
    plot(CurNode(1),CurNode(2),'.','color','red','MarkerSize',20);
    text(CurNode(1) + .1,CurNode(2)+.1,['T_' num2str(iplot)])
    hold on;
        % plot sensing range as circles
    %viscircles(CurNode,SenseRadius,'LineStyle','--');
    hold on;
end  
plot(x(:,1),x(:,2),'k','LineWidth',2.5)