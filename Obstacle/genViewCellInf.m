function [vwCl,vwClLine] = genViewCellInf(nodes,Polys,Obstacle_List,r)
vwClLine = {};
for i = 1:length(nodes)
    node = nodes(i,:);
    %construct 4 triangles
    corners = node + [sqrt(2)*r,0;
        0,sqrt(2)*r;
        -sqrt(2)*r,0;
        0,-sqrt(2)*r];
    polyBox = polyshape(corners(:,1),corners(:,2));
    [neBox,linene] = getViewSeg(Obstacle_List,polyBox,node,corners(1:2,:),r,0,pi/2);
    [nwBox,linnw] = getViewSeg(Obstacle_List,polyBox,node,corners(2:3,:),r,pi/2,pi);
    [swBox,linsw] = getViewSeg(Obstacle_List,polyBox,node,corners(3:4,:),r,-pi,-pi/2);
    [seBox,linse] = getViewSeg(Obstacle_List,polyBox,node,corners([4,1],:),r,-pi/2,0);
    vwClLine{i} = {linene,linnw,linsw,linse};
    tempCel = union(union(neBox,nwBox),union(swBox,seBox));
%     tempCel = intersect(tempCel,Polys);
%     fin = tempCel;
%     if tempCel.NumRegions>1
%         indfin = find(isnan(tempCel.Vertices(:,1)));
%         indfin = [1 indfin'];
%     for k = 2:length(indfin)
%         tempShap = polyshape(tempCel.Vertices(indfin(k-1):indfin(k),1),...
%             tempCel.Vertices(indfin(k-1):indfin(k),2));
%         if inpolygon(node(1),node(2),tempShap.Vertices(:,1),tempShap.Vertices(:,2))
%             fin = tempShap;
%         end
%     end
%     end
%     vwCl(i) =fin;
     vwCl(i) = tempCel;
end




end

function [convView,viewLines] = getViewSeg(Obstacle_List,polyBox1,node,corners,r,t1,t2)
    viewLines = {};
    lineMe = [corners(1,:);corners(2,:)];
    polyBox2 = polyshape([node(1),corners(:,1)'],[node(2),corners(:,2)']);
    %polyBox1 = polyBox2;
    truObs = polyshape.empty;
    truObs2 = truObs;
    totVerts = [];
    for i = 1:length(Obstacle_List)
        testPoly = intersect(polyBox1,Obstacle_List(i));
        testPoly2 = intersect(polyBox2,Obstacle_List(i));
        if testPoly.NumRegions ~= 0
            truObs(end+1) = testPoly;
            %totVerts = [totVerts;testPoly.Vertices];
        end
        if testPoly2.NumRegions ~= 0
            truObs2(end+1) = testPoly2;
            totVerts = [totVerts;testPoly2.Vertices];
        end

    end
    
    if isempty(truObs2)
        convView = polyshape([node(1),corners(:,1)'],[node(2),corners(:,2)']);
        return;
    else
        totVerts = totVerts(~isnan(totVerts(:,1)),:);
    end
    %now we have all the verticies
    firThet = atan2((-node(2)+corners(1,2)),((-node(1)+corners(1,1))));
    firThet = t1;
    endThet = atan2((-node(2)+corners(2,2)),((-node(1)+corners(2,1))));
    endThet = t2;
    allvec = totVerts-node;
    allThet = atan2(allvec(:,2),allvec(:,1))';
    if any(allThet>endThet | allThet<firThet )
        if abs(firThet+pi)<=.000000001
            allThet(abs(allThet-pi)<=.00001) = -pi;
        else
            allThet(abs(allThet-pi)<=.00001) = pi;
        end
    end
    allThet = allThet(allThet<=endThet & allThet>=firThet);
    sTh = unique([allThet,firThet,endThet]);
    %now we should check them in order for points
    thetFin = firThet;
    rFin = 0;
    cartFin = node;
    for i = 1:length(sTh)
        vecOfR = pointsOfInt(r,node,sTh(i),truObs,lineMe);
        curThet = sTh(i);
        lastR  = rFin(end);
        [nVec,indy] = sort([lastR,vecOfR(1,:)]);
        nvec = nVec;
        if indy(1) == 1
            rFin = [rFin, nVec(2:end)];
            for j = 2:length(nVec)
                cartFin = [cartFin;
                   node+nVec(j).*[cos(curThet) sin(curThet)]];
            end
            thetFin = [thetFin,curThet.*ones(1,length(nVec(2:end)))];
        elseif indy(end) == 1
            rFin = [rFin, nVec(end-1:-1:1)];
            thetFin = [thetFin,curThet.*ones(1,length(nVec(end-1:-1:1)))];
            for j = length(nVec)-1:-1:1
                cartFin = [cartFin;
                   node+nVec(j).*[cos(curThet) sin(curThet)]];
            end
        else
            %we have a viewing line not part of the polyshape
            rFin = [rFin, nVec(indy(1)+1:end), nVec(indy(1)-1:-1:1)];
            for j = indy(1)+1:length(nvec)
                cartFin = [cartFin;
                   node+nVec(j).*[cos(curThet) sin(curThet)]];
            end
            for j = indy(1)-1:-1:1
                cartFin = [cartFin;
                   node+nVec(j).*[cos(curThet) sin(curThet)]];
            end
            thetFin = [thetFin,curThet.*ones(1,length(nVec(end-1:-1:1)))];
            rbond = [min(vecOfR(1,:)) max(vecOfR(1,:))];
            points = [node+rbond(1).*[cos(curThet) sin(curThet)];...
                node+rbond(2).*[cos(curThet) sin(curThet)]];
            viewLines{end+1} = points;
        end
    end
    convView = polyshape([cartFin(:,1); node(1)],[cartFin(:,2); node(2)]);
end


function vecOfR = pointsOfInt(r,node,thet,Obstacle_List,line)
    isCon = true; %check all obstacles for interference
    vecOfR = [];
    point1 = node;
    [xi,yi,~] = polyxpoly([node(1), node(1)+ sqrt(2)*r*cos(thet)],[node(2), node(2)+ sqrt(2)*r*sin(thet)],...
        line(:,1),line(:,2));
    point2 = [xi,yi];
    if norm(line(1,:)-(node + sqrt(2)*r*[cos(thet) sin(thet)]))<=.00001
        point2 = line(1,:);
    elseif norm(line(2,:)-(node + sqrt(2)*r*[cos(thet) sin(thet)]))<=.00001
        point2 = line(2,:);
    end
    vecOfR = [norm(node-point2);1];
    for i = 1:length(Obstacle_List)
        probPoints = Obstacle_List(i).Vertices;
        try
        probPoints(end+1,:) = probPoints(1,:);
        catch
            rt = 8+7;
        end
        theta = -thet;
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        p1R = (R*point1')';
        p2R = (R*point2')';
        probPointsR = (R*probPoints' + [0 -p1R(2)]')';
        p1R = [p1R(1),0];
        p2R = [p2R(1),0];
        xlim = [min(probPointsR(:,1)) max(probPointsR(:,1))];
        [xi,yi,seg] = polyxpoly(xlim,[0 0],probPointsR(:,1),probPointsR(:,2));
        if any(abs(probPointsR(:,2))<=.00000000000001)
            cand = find(abs(probPointsR(:,2))<=.00000000000001);
            for j = cand'
                try
                if ~any(abs(xi - probPointsR(j,1))<=.000000001)
                    xi(end+1) = probPointsR(j,1);
                    yi(end+1) = 0;
                end
                catch
                    g = 0;
                end
            end
        end
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
            %poVerInd = seg(k,2);
            if ~any(abs(xi(k) - probPointsR(:,1))<=.001 & abs(yi(k) - probPointsR(:,2))<=.001)
                   valid(k) = 1; %the intersection is NOT a vertex 
                   vecOfR = [vecOfR,[xi(k)-p1R(1);1]];
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
                        vecOfR = [vecOfR,[xi(k)-p1R(1);0]];
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
                            vecOfR = [vecOfR,[xi(k)-p1R(1);0]];
                        else
                            vecOfR = [vecOfR,[xi(k)-p1R(1);1]];
                        end
                    else %we did have s sign change it is va;lid
                        vecOfR = [vecOfR,[xi(k)-p1R(1);1]];
                    end
                end
            end
        end
    end
    %now we have all the R's, with if they are a touch or an intersection
    [~,inds] = sort(vecOfR(1,:));
    vecOfR = vecOfR(:,inds);
    inds = find(vecOfR(2,:));
    vecOfR = vecOfR(:,1:inds(1));
end

