function visbGraph = emVisGraph(TargetNodes,Obstacle_List)
%get the list of locations for the nodes
%first n are targets in indexed order, later nodes are obstacles
nodeLocs = TargetNodes;
for i=1:length(Obstacle_List)
    nodeLocs = [nodeLocs;Obstacle_List(i).Vertices];
end
nodeLocs = nodeLocs(~isnan(nodeLocs(:,1)),:);
visbGraph = graph(zeros(length(nodeLocs)));
visbGraph.Nodes.Location = nodeLocs;
N = height(visbGraph.Nodes);

for i=1:N-1
    for j = i+1:N
        %isCon = noInterfer(visbGraph.Nodes.Location(i,:),visbGraph.Nodes.Location(j,:),Obstacle_List);
        isCon = noInterfer2(visbGraph.Nodes.Location(i,:),visbGraph.Nodes.Location(j,:),Obstacle_List);
%         isCon = true;
%         for l = 1:length(Obstacle_List)
%         [anyIn, ~, ~] = lineinpolygon(visbGraph.Nodes.Location(i,1), visbGraph.Nodes.Location(i,2),...
%             visbGraph.Nodes.Location(j,1), visbGraph.Nodes.Location(j,2), ...
%             Obstacle_List(l).Vertices([1:end,1],1),  Obstacle_List(l).Vertices([1:end,1],2));
%         isCon = ~anyIn & isCon;
         if isCon
            visbGraph = addedge(visbGraph,i,j,norm(visbGraph.Nodes.Location(i,:)-visbGraph.Nodes.Location(j,:)));
        end
    end
       
 end
end



function isCon = noInterfer(point1,point2,Obstacle_List)
    isCon = true; %check all obstacles for interference
    i = 1;
    while i <= length(Obstacle_List) & isCon
        probPoints = Obstacle_List(i).Vertices;
        probPoints(end+1,:) = probPoints(1,:);
%         if Obstacle_List(i).NumHoles > 0 %should just be 1
%             %we just want the verticies after nan
%             ind4hole = find(isnan(Obstacle_List(i).Vertices(:,1)));
%             probPoints = Obstacle_List(i).Vertices(ind4hole(1)+1:end,:);
%             probPoints(end+1,:) = probPoints(1,:);
%         end
        theta = -atan2(point2(2)-point1(2),point2(1)-point1(1));
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        p1R = (R*point1')';
        p2R = (R*point2')';
        probPointsR = (R*probPoints' + [0 -p1R(2)]')';
        p1R = [p1R(1),0];
        p2R = [p2R(1),0];
        xlim = [min(probPointsR(:,1)) max(probPointsR(:,1))];
        [xi,yi,seg] = polyxpoly(xlim,[0 0],probPointsR(:,1),probPointsR(:,2));
        if ~isempty(xi)
            try
            xi = sort(xi);
            startRepeat = [0; abs(diff(xi))<=.001] & abs(xi - probPointsR(1,1))<=.001;
            xi = xi(~startRepeat);
            catch
            end
        end
        valid = ones(1,length(xi));  
        for k = 1:length(xi)
            poVerInd = seg(k,2);
            if ~any(abs(xi(k) - probPointsR(:,1))<=.001 & abs(yi(k) - probPointsR(:,2))<=.001)
                   valid(k) = 1; %the intersection is NOT a vertex 
            else %we are at a vertex
                %just use find/index to 
                poVerInd = find(abs(xi(k) - probPointsR(:,1))<=.001 & abs(yi(k) - probPointsR(:,2))<=.001);
                if all(poVerInd == [1 length(probPointsR(:,1))]')
                    pri = length(probPointsR)-1;
                    aft = 2;
%                 elseif any(poVerInd' == [1 length(probPointsR(:,1))])
%                     if k == poVerInd(1)
%                         valid(k) = 0; %only detect one of the end, but need 
%                         pri = 0;
%                     else
%                        pri = length(probPointsR)-1;
%                        aft = 2;
%                     end
                else
                    pri = poVerInd-1;
                    aft = poVerInd+1;
                end
                if pri ~=0
                    y1 = probPointsR(pri,2);
                    y2 = probPointsR(poVerInd,2);
                    y3 = probPointsR(aft,2);
%                     if (sign(y1) == sign(y2) || sign(y1) == 0 || sign(y2) ==0)...
%                             && (sign(y3) == sign(y2) || sign(y3) == 0 || sign(y3) ==0)
                    if sign(y1) == sign(y3) | (abs(y1)<=.0001 & abs(y3)<=.0001)
                        valid(k) = 0; %this point does NOT count
                    elseif abs(y1)<=.0001 | abs(y3)<=.0001
                        upper = intersect(polyshape(probPointsR(:,1),probPointsR(:,2)),...
                            polyshape([xlim(1) xlim(1) xlim(2) xlim(2)],...
                            [0 max(probPointsR(:,2)) max(probPointsR(:,2)) 0]));
                        lower = intersect(polyshape(probPointsR(:,1),probPointsR(:,2)),...
                            polyshape([xlim(1) xlim(1) xlim(2) xlim(2)],...
                            [0 min(probPointsR(:,2)) min(probPointsR(:,2)) 0]));
                        inUp = inpolygon(xi(k),0,upper.Vertices(:,1),upper.Vertices(:,2));
                        inLow = inpolygon(xi(k),0,lower.Vertices(:,1),lower.Vertices(:,2));
                        if ~(inUp && inLow)
                            valid(k) = 0;
                        end
                    end
                end
            end
        end
        xi_tru = xi(boolean(valid));
        xi_tru = sort(xi_tru);
        %WILL be even or else 
        fail = false;
        if ~(mod(length(xi_tru),2)==0)
            %we will have the start vertex appearing 2x when shopuldnd
            try
            inds = find(xi_tru == probPointsR(1,1));
            xi_tru(inds(end)) = [];
            catch
                xi_tru = [0 0];
                fail = true;
            end
        end
        interSectPairs = zeros(length(xi_tru)/2,2);
        for t = 1:2:length(xi_tru)
            interSectPairs(t,:) = [xi_tru(t) xi_tru(t+1)];
        end
        q1 = p1R(1);
        q2 = p2R(1);
        for t = 1:size(interSectPairs,1)
            p1 = interSectPairs(t,1);
            p2 = interSectPairs(t,2);
            if fail
                isCon = true;
            elseif q1<=p1 & q2>p1
                isCon = false;
            elseif q1>p1 & q1<p2
                isCon = false;
            end
        end
%         probPoints(end,:) = []; %now remove
%         vertInds = [];
%         for k = 1:length(xi)
%             inds = find((xi(k) == probPoints(:,1)) & (yi(k) == probPoints(:,2)));
%             if isempty(inds)
%                 isCon = false; %if the overlaps are not at a corner it's over
% %             elseif length(inds)>1 
% %                 isCon = false; %if the overlap point is a double vertex, it is over
%             end
%             vertInds(end+1) = inds;
%         end %if no intersections, never have chance to change to false
%         %now we have to check if we had two verticies that were not aligned
%         if isCon
%             %we check if the the points are visited in order
%             myDif = diff(vertInds);
%             if all(abs(myDif) == 1 || all(abs(myDif) == length(probPoints)-1))
%                 %we are ok because they are neighbors
%             else %need to check seperating hyperplane
% 
%             end
%         end
        i = i+1;
    end
end


function isCon = noInterfer2(point1,point2,Obstacle_List)
isCon = true;
run = point2(1)-point1(1);
rise = point2(2)-point1(2);
myVec = [-rise run];
polyShape1 = polyshape([point1(1) point1(1)+ myVec(1) point2(1)+ myVec(1) point2(1)],...
    [point1(2) point1(2)+ myVec(2) point2(2)+ myVec(2) point2(2)]);
polyShape2 = polyshape([point1(1) point1(1)- myVec(1) point2(1)- myVec(1) point2(1)],...
    [point1(2) point1(2)- myVec(2) point2(2)- myVec(2) point2(2)]);
for i = 1:length(Obstacle_List)
    allObs = Obstacle_List(i);
    upperObs = intersect(polyShape1,allObs);
    lowerObs = intersect(polyShape2,allObs);
    curVer = [];
    currentn1 = allObs.Vertices(1,:);
    for k = 1:length(allObs.Vertices)
        if ~isnan(allObs.Vertices(k,1))
            curVer(end+1,:) = allObs.Vertices(k,:);
        else
            curVer(end+1,:) = currentn1;
            curVer(end+1,:) = nan;
            currentn1 = allObs.Vertices(k+1,:); %no nan at end 
        end
    end
    curVer(end+1,:) = currentn1;
    %now we have list of interior and exterior edges 
    [xi,yi,~] = polyxpoly([point1(1) point2(1)],[point1(2) point2(2)],curVer(:,1),curVer(:,2));
    %if xi and yi are empty, its either fully inside or outside,easy check:
    sampleMid = (point1+point2)/2;
    [in, on] = inpolygon(sampleMid(1),sampleMid(2),allObs.Vertices(:,1),allObs.Vertices(:,2));
    if in && ~on
        isCon = false;
    end

    points2check = [xi,yi];
    for j = 1:length(points2check(:,1))
        if inpolygon(points2check(j,1),points2check(j,2),upperObs.Vertices(:,1),upperObs.Vertices(:,2))&&...
                inpolygon(points2check(j,1),points2check(j,2),lowerObs.Vertices(:,1),lowerObs.Vertices(:,2))
            isCon = false;
        end
    end

    %check if any points to check is a vertex on both obstacles-make sure
    %point1 and 2 are ignored
    logIndUp = ~(upperObs.Vertices(:,1) == point1(1) & upperObs.Vertices(:,2) == point1(2));
    logIndUp = logIndUp & ~(upperObs.Vertices(:,1) == point2(1) & upperObs.Vertices(:,2) == point2(2));
    logIndLow = ~(lowerObs.Vertices(:,1) == point1(1) & lowerObs.Vertices(:,2) == point1(2));
    logIndLow = logIndLow & ~(lowerObs.Vertices(:,1) == point2(1) & lowerObs.Vertices(:,2) == point2(2));
    legitVertU = upperObs.Vertices(logIndUp,:);
    legitVertL = lowerObs.Vertices(logIndLow,:);
    for j = 1:length(legitVertU(:,1))
        for k = 1:length(legitVertL(:,1))
            if norm(legitVertU(j,:)-legitVertL(k,:))<=eps*1000000
                isCon = false;
            end
        end
    end


end

end
