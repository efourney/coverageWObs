function Obstacle_List = placeIntersPointsLineNPoly(x1,y1,Obstacle_List)
for i = 1:length(Obstacle_List)
    polyShapeVert = Obstacle_List(i).Vertices;
    if ~isempty(polyShapeVert)
    polyShapeVert(end+1,:) = polyShapeVert(1,:);%add completing line segment
    [xi,yi,link] = polyxpoly(x1,y1,polyShapeVert(:,1),polyShapeVert(:,2),'unique');
    [link,ind] = sort(link(:,2));
    xi = xi(ind);
    yi = yi(ind);
    for j = 1:size(link,1)
        indOS = link(j);
        polyShapeVert = [polyShapeVert(1:indOS,:);xi(j) yi(j);polyShapeVert(indOS+1:end,:)];
        link = link+1; %account for shifting of verticies
    end
    Obstacle_List(i) = polyshape(polyShapeVert(:,1),polyShapeVert(:,2),'KeepCollinearPoints',true);
    end
end
end

