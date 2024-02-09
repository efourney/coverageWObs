function isCon = noInterferObsLinSeg(point1,point2,Obstacle_List)
  isCon = true; %check all obstacles for interference
    i = 1;
    while i <= length(Obstacle_List) & isCon
        probPoints = Obstacle_List(i).Vertices;
        probPoints(end+1,:) = probPoints(1,:);
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
        if ~(mod(length(xi_tru),2)==0)
            %we will have the start vertex appearing 2x when shopuldnd
            inds = find(xi_tru == probPointsR(1,1));
            xi_tru(inds(end)) = [];
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
            if q1<=p1 & q2>p1
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

