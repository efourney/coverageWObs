function [vwCl,vwClLine,thetCl,oclRay] = genViewCellInfTri(nodes,Polys,Obstacle_List,r)
vwClLine = {};
oclRay = {};
thetCl = {};
vwClLine = {};
for i = 1:length(nodes(:,1))
    node = nodes(i,:);
    %construct 4 triangles
    corners = node + [sqrt(2)*r,0;
        0,sqrt(2)*r;
        -sqrt(2)*r,0;
        0,-sqrt(2)*r];
    polyBox = polyshape(corners(:,1),corners(:,2));
    [swBox,linsw,obsThisCirc] = getViewSeg(Obstacle_List,polyBox,node,corners(3:4,:),r,-pi,-pi/2);
    [seBox,linse,~] = getViewSeg(Obstacle_List,polyBox,node,corners([4,1],:),r,-pi/2,0);
    [neBox,linene,~] = getViewSeg(Obstacle_List,polyBox,node,corners(1:2,:),r,0,pi/2);
    [nwBox,linnw,~] = getViewSeg(Obstacle_List,polyBox,node,corners(2:3,:),r,pi/2,pi);
    vwClLine{i} = {linene,linnw,linsw,linse};
    tempCel = union(union(neBox,nwBox),union(swBox,seBox));
    tempCel = intersect(polyBox,tempCel);
    tempCel = intersect(polyBox,tempCel);
    obsCellVert = {};
    for m = 1:length(obsThisCirc)
        curMat = obsThisCirc(m).Vertices;
        obsCellVert{end+1} = [curMat;curMat(1,:)];
    end
    %we generate lists of theta where the circle is uniterupoted
    %we can take the tempCel and check which veritcies are further than r
    %away
    [tcX,tcY] = poly2ccw(tempCel.Vertices(:,1),tempCel.Vertices(:,2));
    tcX = [tcX; tcX(1)];
    tcY = [tcY; tcY(1)];
    thetCur = [];
    oclrM = [];
    for j = 1:(length(tcX)-1)
        p1 = [tcX(j) tcY(j)];
        p2 = [tcX(j+1) tcY(j+1)];
        if norm(p1-node)>=r-eps*1000 & norm(p2-node)>=r-eps*1000
            [points] = pointsIntLineSegCirc(p1,p2,node,r);
            if size(points,1)<=1
                %we have a range of thetas
                vec1 = p1-node;
                vec2 = p2-node;
                if atan2(vec1(2),vec1(1))>0 & atan2(vec2(2),vec2(1))<0
                    thetCur(end+1,:) = [atan2(vec1(2),vec1(1)),pi];
                    thetCur(end+1,:) = [-pi,atan2(vec2(2),vec2(1))];
                else
                    thetCur(end+1,:) = [atan2(vec1(2),vec1(1)),atan2(vec2(2),vec2(1))];
                end
            end
        end
        %now look at if we should include as viable occlusion rays
        arTri = .5*abs(node(1)*(p1(2)-p2(2))+p1(1)*(p2(2)-node(2))+p2(1)*(node(2)-p1(2)));
        if arTri<=(r^2)/500000 %will be 0 if it is a line,but account for math errors
            [points] = pointsIntLineSegCirc(p1,p2,node,r);
            if size(points,1)>1
                %check if this is an obstracle edge
                obsEdge = false;
                for p = 1:length(obsCellVert)
                    curMat = obsCellVert{p};
                    indsP1 = find(abs(curMat(:,1) - p1(1))<= eps*10000 & abs(curMat(:,2)- p1(2))<= eps*10000);
                    indsP2 = find(abs(curMat(:,1) - p2(1))<= eps*10000 & abs(curMat(:,2) - p2(2))<= eps*10000);
                    if ~isempty(indsP1) && ~isempty(indsP2)
                        for m = 1:length(indsP1)
                            vecOfDif = indsP2-indsP1(m);
                            if any(abs(vecOfDif)==1)
                                obsEdge = true;
                            end
                        end
                    end
                end
                if ~obsEdge
                    oclrM(end+1,:) = [points(1,:) points(2,:)];
                end
            end
        end
    end
    thetCl{end+1} = thetCur;
    oclRay{end+1} = oclrM;
  


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
corners = node + [r,-r;
        r,r;
        -r,r;
        -r,-r];
    polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,tempCel);
     corners = node'+ sqrt(2)/2.*[1 -1; 1 1]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
     polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
     thet = pi/3;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
     polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
          thet = 2*pi/3;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
     polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
     thet = pi/6;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
     polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
     thet = 4*pi/6;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
      polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
     thet = 3.5*pi/6;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
      polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
     thet = 5.5*pi/6;
     corners = node'+ [cos(thet) -sin(thet);sin(thet) cos(thet)]*([r,-r;...
        r,r;...
        -r,r;...
        -r,-r]');
     corners = corners';
      polyBox2 = polyshape(corners(:,1),corners(:,2));
     vwCl(i) = intersect(polyBox2,vwCl(i));
end




end

function [convView,viewLines,truObs] = getViewSeg(Obstacle_List,polyBox1,node,corners,r,t1,t2)
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
            %truObs(end+1) = testPoly;
            truObs(end+1) = Obstacle_List(i);
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
    %now we make each seperate island its own obs
%     truObsReal = polyshape.empty;
%     for k = 1:length(truObs)
%         if truObs(k).NumRegions>1
%             lastInd = 1;
%             for o = 1:length(truObs(k).Vertices)
%                 if isnan(truObs(k).Vertices(o,1)) 
%                     truObsReal(end+1) = polyshape(truObs(k).Vertices(lastInd:o-1,1),...
%                         truObs(k).Vertices(lastInd:o-1,2));
%                     lastInd = o + 1;
%                 elseif o == length(truObs(k).Vertices)
%                     truObsReal(end+1) = polyshape(truObs(k).Vertices(lastInd:o-1,1),...
%                         truObs(k).Vertices(lastInd:o-1,2));
%                 end
%             end
%         else
%             truObsReal(end+1) = truObs(k);
%         end
%     end
%     truObs = truObsReal;



    %now we have all the verticies
    firThet = t1;
    endThet = t2;
    allvec = totVerts-node;
    allThet = atan2(allvec(:,2),allvec(:,1))';
    %since +/-pi are the same, switch to what we care about
    if any(allThet>endThet | allThet<firThet ) %we are always using corners
        if any([firThet endThet] == pi)
            allThet(abs(allThet+pi)<=.00001) = pi;
        else
            allThet(abs(allThet-pi)<=.00001) = -pi;
        end
    end
    allThet = allThet(allThet<=endThet & allThet>=firThet);
    sTh = uniquetol([allThet,firThet,endThet]);
    %now we should check them in order for points
    %hack for starying poly list
    viewTriList = [polyshape([0 1 2],[0 1 0]),polyshape([0 1 2],[0 1 0])];
    for i = 1:length(sTh)-1
        [vecOfR1, side1]= pointsOfInt(r,node,sTh(i),truObs,lineMe);
        [vecOfR2, side2] = pointsOfInt(r,node,sTh(i+1),truObs,lineMe);
%         side1 = side1(vecOfR1(1,:)>=0);
%         vecOfR1 = vecOfR1(vecOfR1(1,:)>=0,:);
%         side2 = side2(vecOfR2(1,:)>=0);
%         vecOfR2 = vecOfR2(vecOfR2(1,:)>=0,:);
        realInd1 = side1 ~= 'l' & side1 ~= 'c' ;
        vecOfR1 = vecOfR1(:,realInd1);
        side1 = side1(realInd1);
        realInd2 = side2 ~= 'u' & side2 ~= 'c' ;
        vecOfR2 = vecOfR2(:,realInd2);
        side2 = side2(realInd2);
        try
        verts = [node;
            node + vecOfR1(1).*[cos(sTh(i)) sin(sTh(i))];
            node + vecOfR2(1).*[cos(sTh(i+1)) sin(sTh(i+1))]];
        catch
            1+1
        end
        viewTriList(end+1) = polyshape(verts(:,1),verts(:,2));

%         curThet = sTh(i);
%         lastR  = rFin(end);
%         [nVec,indy] = sort([lastR,vecOfR(1,:)]);
%         nvec = nVec;
%         if indy(1) == 1
%             rFin = [rFin, nVec(2:end)];
%             for j = 2:length(nVec)
%                 cartFin = [cartFin;
%                    node+nVec(j).*[cos(curThet) sin(curThet)]];
%             end
%             thetFin = [thetFin,curThet.*ones(1,length(nVec(2:end)))];
%         elseif indy(end) == 1
%             rFin = [rFin, nVec(end-1:-1:1)];
%             thetFin = [thetFin,curThet.*ones(1,length(nVec(end-1:-1:1)))];
%             for j = length(nVec)-1:-1:1
%                 cartFin = [cartFin;
%                    node+nVec(j).*[cos(curThet) sin(curThet)]];
%             end
%         else
%             %we have a viewing line not part of the polyshape
%             rFin = [rFin, nVec(indy(1)+1:end), nVec(indy(1)-1:-1:1)];
%             for j = indy(1)+1:length(nvec)
%                 cartFin = [cartFin;
%                    node+nVec(j).*[cos(curThet) sin(curThet)]];
%             end
%             for j = indy(1)-1:-1:1
%                 cartFin = [cartFin;
%                    node+nVec(j).*[cos(curThet) sin(curThet)]];
%             end
%             thetFin = [thetFin,curThet.*ones(1,length(nVec(end-1:-1:1)))];
%             rbond = [min(vecOfR(1,:)) max(vecOfR(1,:))];
%             points = [node+rbond(1).*[cos(curThet) sin(curThet)];...
%                 node+rbond(2).*[cos(curThet) sin(curThet)]];
%             viewLines{end+1} = points;
%         end
    end
    viewTriList = viewTriList(3:end); %getting rid of dummy starts
    convView = viewTriList(1);
    for k = 2:length(viewTriList)
        convView = union(convView,viewTriList(k));
    end
end


function [vecOfR, side] = pointsOfInt(r,node,thet,Obstacle_List,line)
    isCon = true; %check all obstacles for interference
    vecOfR = [];
    side = [];
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
    side = ['e'];
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
                   side = [side,'b']; %both sides
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
                    if sign(y1) == sign(y3) | (abs(y1)<=.000001 & abs(y3)<=.000001)
                        valid(k) = 0; %this point does NOT count
                        vecOfR = [vecOfR,[xi(k)-p1R(1);0]];
                        if y1>0
                            side = [side,'u']; 
                        elseif y1<0
                            side = [side,'l']; 
                        else
                            side = [side,'c']; 
                        end
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
                            if inUp
                                side = [side,'u'];
                            elseif inLow
                                side = [side,'l'];
                            else
                                inUp1 = inpolygon(xi(k)-.00001,.000000001,upper.Vertices(:,1),upper.Vertices(:,2));
                                inUp2 = inpolygon(xi(k),.0001,upper.Vertices(:,1),upper.Vertices(:,2));
                                inUp3 = inpolygon(xi(k)+.00001,.0000000001,upper.Vertices(:,1),upper.Vertices(:,2));
                                inl1 = inpolygon(xi(k)-.0001,-.0000000001,lower.Vertices(:,1),lower.Vertices(:,2));
                                inl2 = inpolygon(xi(k),-.000001,lower.Vertices(:,1),lower.Vertices(:,2));
                                inl3 = inpolygon(xi(k)+.00001,-.0000000001,lower.Vertices(:,1),lower.Vertices(:,2));
                                if inUp1 || inUp2 || inUp3
                                    side = [side,'u'];
                                elseif inl1 || inl2 || inl3
                                    side = [side,'l'];
                                else
                                    side = 'h'
                                end
                            end
                        else
                            vecOfR = [vecOfR,[xi(k)-p1R(1);1]];
                            side = [side,'b']; 
                        end
                    else %we did have s sign change it is va;lid
                        vecOfR = [vecOfR,[xi(k)-p1R(1);1]];
                        side = [side,'b']; 
                    end
                end
            end
        end
    end
    %now we have all the R's, with if they are a touch or an intersection
    [~,inds] = sort(vecOfR(1,:));
    vecOfR = vecOfR(:,inds);
    side = side(inds);
    inds = vecOfR(1,:)>=0;
    vecOfR = vecOfR(:,inds);
    side = side(inds);
    inds = find(vecOfR(2,:));
    vecOfR = vecOfR(:,1:inds(1));
    side = side(1:inds(1));
end

