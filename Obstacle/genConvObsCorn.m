function box3 = genConvObsCorn(cornerPoint,othrPoint,Obstacle_List,boundingBox)
Obstacle_coords = [];
for i = 1:length(Obstacle_List)
    curver = Obstacle_List(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end

indOfCorner = find(cornerPoint(1) == Obstacle_coords(:,1) & cornerPoint(2) == Obstacle_coords(:,2));
if length(indOfCorner) > 1
    Obstacle_coords(indOfCorner(2:end),:) = []; %eliminate duplicates
end
obstacle_num = Obstacle_coords(indOfCorner(1),3);

Obstacle_coords = Obstacle_coords(Obstacle_coords(:,3) == obstacle_num,:);

indOfCorner = find(cornerPoint(1) == Obstacle_coords(:,1) & cornerPoint(2) == Obstacle_coords(:,2));
%now definitely only one, only care about THIS obstacle
if indOfCorner == 1
    cornera = Obstacle_coords(end,1:2);
    cornerb = Obstacle_coords(2,1:2);
elseif indOfCorner == length(Obstacle_coords)
    cornera = Obstacle_coords(end-1,1:2);
    cornerb = Obstacle_coords(1,1:2);
else
    cornera = Obstacle_coords(indOfCorner-1,1:2);
    cornerb = Obstacle_coords(indOfCorner+1,1:2);
end

vecLine = othrPoint - cornerPoint;
obsLinea = cornera - cornerPoint;
obsLineb = cornerb - cornerPoint;

cosAngA = dot(vecLine,obsLinea)./(norm(vecLine)*norm(obsLinea));
cosAngB = dot(vecLine,obsLineb)./(norm(vecLine)*norm(obsLineb));

%hyperplane points
if cosAngA>cosAngB
    hypPoin2 = cornera;
else
    hypPoin2 = cornerb;
end

%now we have 2 points. We can use these to construct a hyperplane with
%othrepoint included, but functionally we can use polygon objects.
box = polyshape(boundingBox(:,1),boundingBox(:,2));

% get intersections of line segment with polygon as if it extended 
point1 = cornerPoint;
point2 = hypPoin2;

rise = point2(2)-point1(2);
run = point2(1)-point1(1);

xlim = [min(boundingBox(:,1)) max(boundingBox(:,1))];
ylim = [min(boundingBox(:,2)) max(boundingBox(:,2))];
if rise == 0 && run~=0
    %stright line at y = point1(2)
    x1 = xlim;
    y1 = [point1(2) point1(2)];
    num1 = 0;
    cutPoly1 = [x1(1), y1(1) ;x1(1), ylim(2);x1(2), ylim(2);x1(2), y1(1)];
    cutPoly2 = [x1(1), y1(1) ;x1(1), ylim(1);x1(2), ylim(1);x1(2), y1(1)];
elseif run == 0 && rise~=0
    x1 = [point1(1) point1(1)];
    y1 = ylim;
    num1 = 0;
    cutPoly1 = [x1(1), ylim(1) ;x1(1), ylim(2);xlim(2), ylim(2);xlim(2), ylim(1)];
    cutPoly2 = [x1(1), ylim(1) ;x1(1), ylim(2);xlim(1), ylim(2);xlim(1), ylim(1)];
elseif run == 0 && rise==0
    x1 = point1(1);
    y1 = point1(2);
%ignored
else
    slope = rise/run;
    x1 = [point1(1)-(xlim(2)-xlim(1)) point2(1)+(xlim(2)-xlim(1))];
    y1 = [point1(2)-slope*(xlim(2)-xlim(1)) point2(2)+slope*(xlim(2)-xlim(1))];
    num1 = 1;
end

[xi,yi] = polyxpoly(x1,y1,boundingBox(:,1),boundingBox(:,2));
xi = x1';
yi = y1';

if num1 ~=0
cutPoly1 = cutpolygon(boundingBox,[xi,yi],num1,false,false,false);
cutPoly2 = cutpolygon(boundingBox,[xi,yi],num1+1,false,false,false);
end

if inpolygon(othrPoint(1),othrPoint(2),cutPoly1(:,1),cutPoly1(:,2))
    firstBox = polyshape(cutPoly1(:,1),cutPoly1(:,2));
else
    firstBox = polyshape(cutPoly2(:,1),cutPoly2(:,2));
end

for i = 1:length(Obstacle_List)
    Obstacle_List(i) = intersect(Obstacle_List(i),firstBox,'KeepCollinearPoints',true);
end

Obstacle_List = Obstacle_List([Obstacle_List.NumRegions] ~= 0);


box1 = genConvCellyfirst(cornerPoint,othrPoint,Obstacle_List,boundingBox);
box2 = genConvCellxfirst(cornerPoint,othrPoint,Obstacle_List,boundingBox);

box1t = intersect(box1,firstBox,'KeepCollinearPoints',true);
box2t = intersect(box2,firstBox,'KeepCollinearPoints',true);

if area(box1t) > area(box2t)
    box3 = box1t;
else
    box3 = box2t;
end

end