function box = genConvCellxfirst(point1,point2,Obstacle_List,boundingBox)
box = polyshape(boundingBox(:,1),boundingBox(:,2));

if isempty(Obstacle_List)
    return
end

% get intersections of line segment with polygon as if it extended 
rise = point2(2)-point1(2);
run = point2(1)-point1(1);

xlim = [min(boundingBox(:,1)) max(boundingBox(:,1))];
ylim = [min(boundingBox(:,2)) max(boundingBox(:,2))];
if rise == 0 && run~=0
    %stright line at y = point1(2)
    x1 = xlim;
    y1 = [point1(2) point1(2)];
    x2 = [point1(1) point1(1)];
    y2 = ylim;
    x3 = [point2(1) point2(1)];
    y3 = ylim;
    num1 = 2;
    num2 = 4;
    num3 = 4;
elseif run == 0 && rise~=0
    x1 = [point1(1) point1(1)];
    y1 = ylim;
    x2 = xlim;
    x3=xlim;
    y2=[point1(2) point1(2)];
    y3=[point2(2) point2(2)];
    num1 = 4;
    num2 = 1;
    num3 = 1;
elseif run == 0 && rise==0
    x1 = point1(1);
    y1 = point1(2);
    x2 = point1(1);
    y2 = point1(2);
    x3 = point1(1);
    y3 = point1(2);
else
    slope = rise/run;
    x1 = [point1(1)-(xlim(2)-xlim(1)) point2(1)+(xlim(2)-xlim(1))];
    y1 = [point1(2)-slope*(xlim(2)-xlim(1)) point2(2)+slope*(xlim(2)-xlim(1))];
    slopeOth = -run/rise;
    x2 = [point1(1)-(xlim(2)-xlim(1)) point1(1)+(xlim(2)-xlim(1))];
    y2 = [point1(2)-slopeOth*(xlim(2)-xlim(1)) point1(2)+slopeOth*(xlim(2)-xlim(1))];
    x3 = [point2(1)-(xlim(2)-xlim(1)) point2(1)+(xlim(2)-xlim(1))];
    y3 = [point2(2)-slopeOth*(xlim(2)-xlim(1)) point2(2)+slopeOth*(xlim(2)-xlim(1))];
    num1 = 1;
    num2 = 2;
    num3 = 3;
end

[xi,yi] = polyxpoly(x1,y1,boundingBox(:,1),boundingBox(:,2));
xi = x1';
yi = y1';

%taken points are collinear from matlab help pointsAreCollinear = @(xy) rank(xy(2:end,:) - xy(1,:)) == 1;

% for i = 1:length(Obstacle_List)
%     
%     cutPoly = cutpolygon(Obstacle_List(i).Vertices,[xi,yi],num1,true,false,false);
%     Obstacle_List(i) = union(polyshape(cutPoly(:,1),cutPoly(:,2)),Obstacle_List(i),'KeepCollinearPoints',true);
%     cutPoly = cutpolygon(Obstacle_List(i).Vertices,[x2',y2'],num2,true,false,false);
%     Obstacle_List(i) = union(polyshape(cutPoly(:,1),cutPoly(:,2)),Obstacle_List(i),'KeepCollinearPoints',true);
%     cutPoly = cutpolygon(Obstacle_List(i).Vertices,[x3',y3'],num3,true,false,false);
%     Obstacle_List(i) = union(polyshape(cutPoly(:,1),cutPoly(:,2)),Obstacle_List(i),'KeepCollinearPoints',true);
% end


Obstacle_List = placeIntersPointsLineNPoly(xi,yi,Obstacle_List);
Obstacle_List = placeIntersPointsLineNPoly(x2,y2,Obstacle_List);
Obstacle_List = placeIntersPointsLineNPoly(x3,y3,Obstacle_List);


Obstacle_coords = [];
for i = 1:length(Obstacle_List)
    curver = Obstacle_List(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end

theta = -atan2(point2(2)-point1(2),point2(1)-point1(1));
R = [cos(theta) -sin(theta); sin(theta) cos(theta)]';
Obstacle_coords_rot = [Obstacle_coords(:,1:2)*R,Obstacle_coords(:,3)];
theta = -1*theta;
R_back = [cos(theta) -sin(theta); sin(theta) cos(theta)]';
point1_rot = point1*R;
point2_rot = point2*R;
Obstacle_coords_rot_sorting = [Obstacle_coords_rot;[point1_rot,0];[point2_rot,0]];

%get og boundingBox bounds
boundingBoxRot = boundingBox*R;
xlim = [min(boundingBoxRot(:,1)) max(boundingBoxRot(:,1))];
ylim = [min(boundingBoxRot(:,2)) max(boundingBoxRot(:,2))];



%first move out in x direction
[~,sortedInds] = sort(Obstacle_coords_rot_sorting(:,1));
Obstacle_coords_rot_sorting = Obstacle_coords_rot_sorting(sortedInds,:);
inds = find(Obstacle_coords_rot_sorting(:,3)==0);
myInd = inds(1); %first one is furthest left/lowest
incidence = find(Obstacle_coords_rot_sorting(:,1)==Obstacle_coords_rot_sorting(myInd,1)&...
    Obstacle_coords_rot_sorting(:,3)~=0);
outerCornerFound = false;
if myInd == 1
        %no obstacles!
        outerCornerFound = true;
elseif ~isempty(incidence)
    probPoint = Obstacle_coords_rot_sorting(incidence(1),1:2);
    box_rotated_vert = box.Vertices*R;
    xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
    ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
    %         cutBox = cutpolygon(box_rotated_vert,[[probPoint(1),probPoint(1)]',[min(box_rotated_vert(:,2)),max(box_rotated_vert(:,2))]'],...
    %             3,true,false,false);
    cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
        polyshape([probPoint(1) probPoint(1) xlimrot(2) xlimrot(2)],...
        [ylimrot(1) ylimrot(2) ylimrot(2) ylimrot(1)]));
    cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
        polyshape([probPoint(1) probPoint(1) xlim(2) xlim(2)],...
        [ylim(1) ylim(2) ylim(2) ylim(1)]));
    cutBox = cutBox.Vertices;
    %cut box is a polygon in rotated coordinates carrying the safe set
    cutBox_ogframe = cutBox*R_back;
    box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
    outerCornerFound = true;
else
    guess = myInd-1;
end
while ~outerCornerFound
    %there is something to left of leftmost point
    obstacleInGuess = Obstacle_coords_rot_sorting(guess,3);
    %find x coordinates of all points for obstacle in guess
    indsOfGuess = find(Obstacle_coords_rot_sorting(:,3) == obstacleInGuess);
    if all(indsOfGuess>=guess)
        %these x values all greater than our guess point so we found the
        %end of the obstacle, this point is not a valid constraint
        guess = guess-1;
        if guess == 0
            outerCornerFound = true; %no restraint
        elseif Obstacle_coords_rot_sorting(guess,3) == 0
            guess = guess-1;
        end
    else
        %we know what point is the problem
        outerCornerFound = true;
        %we need to rotate the box into our frame
        probPoint = Obstacle_coords_rot_sorting(guess,1:2);
        box_rotated_vert = box.Vertices*R;
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
%         cutBox = cutpolygon(box_rotated_vert,[[probPoint(1),probPoint(1)]',[min(box_rotated_vert(:,2)),max(box_rotated_vert(:,2))]'],...
%             3,true,false,false);
       cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([probPoint(1) probPoint(1) xlimrot(2) xlimrot(2)],...
            [ylimrot(1) ylimrot(2) ylimrot(2) ylimrot(1)]));
       cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([probPoint(1) probPoint(1) xlim(2) xlim(2)],...
            [ylim(1) ylim(2) ylim(2) ylim(1)]));
        cutBox = cutBox.Vertices;
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
    end
end

%now we want to only care about the partof other shapes in the box, so we
%go down the list and intersect the shapes with the convex cell

Obstacle_coords = [];
Obstacle_List_intersected = Obstacle_List;
for i = 1:length(Obstacle_List)
    Obstacle_List_intersected(i) = intersect(Obstacle_List(i),box,'KeepCollinearPoints',true);
    curver = Obstacle_List_intersected(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end

%first move out in x direction - right
Obstacle_coords_rot = [Obstacle_coords(:,1:2)*R,Obstacle_coords(:,3)];
Obstacle_coords_rot_sorting = [Obstacle_coords_rot;[point1_rot,0];[point2_rot,0]];

[~,sortedInds] = sort(Obstacle_coords_rot_sorting(:,1));
Obstacle_coords_rot_sorting = Obstacle_coords_rot_sorting(sortedInds,:);
inds = find(Obstacle_coords_rot_sorting(:,3)==0);
myInd = inds(2); %first one is furthest left/lowest, 2nd is furthest right
incidence = find(Obstacle_coords_rot_sorting(:,1)==Obstacle_coords_rot_sorting(myInd,1)&...
    Obstacle_coords_rot_sorting(:,3)~=0);
outerCornerFound = false;
if myInd == length(Obstacle_coords_rot_sorting(:,3))
        %no obstacles!
        outerCornerFound = true;
elseif ~isempty(incidence)
    probPoint = Obstacle_coords_rot_sorting(incidence(1),1:2);
        %we know what point is the problem
        outerCornerFound = true;
        %we need to rotate the box into our frame
        box_rotated_vert = box.Vertices*R;
%         cutBox = cutpolygon(box_rotated_vert,[[probPoint(1),probPoint(1)]',[min(box_rotated_vert(:,2)),max(box_rotated_vert(:,2))]'],...
%             4,true,false,false);
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
       cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) probPoint(1) probPoint(1) ],...
            [ylimrot(1) ylimrot(2) ylimrot(2) ylimrot(1)]));
              cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) probPoint(1) probPoint(1) ],...
            [ylim(1) ylim(2) ylim(2) ylim(1)]));
        cutBox = cutBox.Vertices;        
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
else
    guess = myInd+1;
end
while ~outerCornerFound
    %there is something to left of leftmost point
    obstacleInGuess = Obstacle_coords_rot_sorting(guess,3);
    %find x coordinates of all points for obstacle in guess
    indsOfGuess = find(Obstacle_coords_rot_sorting(:,3) == obstacleInGuess);
    if all(indsOfGuess<=guess)
        %these x values all less than our guess point so we found the
        %end of the obstacle, this point is not a valid constraint
        guess = guess+1;
        if guess == length(Obstacle_coords_rot_sorting(:,3))+1
            outerCornerFound = true; %no restraint
        elseif Obstacle_coords_rot_sorting(guess,3) == 0
            guess = guess+1;
        end
    else
        %we know what point is the problem
        outerCornerFound = true;
        %we need to rotate the box into our frame
        probPoint = Obstacle_coords_rot_sorting(guess,1:2);
        box_rotated_vert = box.Vertices*R;
%         cutBox = cutpolygon(box_rotated_vert,[[probPoint(1),probPoint(1)]',[min(box_rotated_vert(:,2)),max(box_rotated_vert(:,2))]'],...
%             4,true,false,false);
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
       cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) probPoint(1) probPoint(1) ],...
            [ylimrot(1) ylimrot(2) ylimrot(2) ylimrot(1)]));
              cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) probPoint(1) probPoint(1) ],...
            [ylim(1) ylim(2) ylim(2) ylim(1)]));
        cutBox = cutBox.Vertices;        
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
    end
end

%now we want to only care about the partof other shapes in the box, so we
%go down the list and intersect the shapes with the convex cell

Obstacle_coords = [];
Obstacle_List_intersected = Obstacle_List_intersected;
for i = 1:length(Obstacle_List)
    Obstacle_List_intersected(i) = intersect(Obstacle_List_intersected(i),box,'KeepCollinearPoints',true);
    curver = Obstacle_List_intersected(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end


%now y and go down
Obstacle_coords_rot = [Obstacle_coords(:,1:2)*R,Obstacle_coords(:,3)];
Obstacle_coords_rot_sorting = [Obstacle_coords_rot;[point1_rot,0];[point2_rot,0]];

%sort by y coordinate
[~,sortedInds] = sort(Obstacle_coords_rot_sorting(:,2));
Obstacle_coords_rot_sorting = Obstacle_coords_rot_sorting(sortedInds,:);
inds = find(Obstacle_coords_rot_sorting(:,3)==0);
myInd = inds(1); %first one is lowest, 2nd is highest 
incidence = find(Obstacle_coords_rot_sorting(:,2)==Obstacle_coords_rot_sorting(myInd,2)&...
    Obstacle_coords_rot_sorting(:,3)~=0);
outerCornerFound = false;
if myInd == 1
        %no obstacles!
        outerCornerFound = true;
elseif ~isempty(incidence)
    outerCornerFound = true;
    probPoint = Obstacle_coords_rot_sorting(incidence(1),1:2);
            box_rotated_vert = box.Vertices*R;
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
%         cutBox = cutpolygon(box_rotated_vert,[[min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))]',[probPoint(2),probPoint(2)]'],...
%             1,true,false,false);
        cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) xlimrot(2) xlimrot(2) ],...
            [probPoint(2) ylimrot(2) ylimrot(2) probPoint(2)]));
                cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) xlim(2) xlim(2) ],...
            [probPoint(2) ylim(2) ylim(2) probPoint(2)]));
        cutBox = cutBox.Vertices;   
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
else
    guess = myInd-1;
end
while ~outerCornerFound
    %there is something below the lower point
    obstacleInGuess = Obstacle_coords_rot_sorting(guess,3);
    %find y coordinates of all points for obstacle in guess
    indsOfGuess = find(Obstacle_coords_rot_sorting(:,3) == obstacleInGuess);
    if all(indsOfGuess>=guess)
        %these x values all less than our guess point so we found the
        %end of the obstacle, this point is not a valid constraint
        guess = guess-1;
        if guess == 0
            outerCornerFound = true; %no restraint
        elseif Obstacle_coords_rot_sorting(guess,3) == 0
             guess = guess-1;
        end
    else
        %we know what point is the problem
        outerCornerFound = true;
        %we need to rotate the box into our frame
        probPoint = Obstacle_coords_rot_sorting(guess,1:2);
        box_rotated_vert = box.Vertices*R;
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
%         cutBox = cutpolygon(box_rotated_vert,[[min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))]',[probPoint(2),probPoint(2)]'],...
%             1,true,false,false);
        cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) xlimrot(2) xlimrot(2) ],...
            [probPoint(2) ylimrot(2) ylimrot(2) probPoint(2)]));
                cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) xlim(2) xlim(2) ],...
            [probPoint(2) ylim(2) ylim(2) probPoint(2)]));
        cutBox = cutBox.Vertices;   
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
    end
end

%now we want to only care about the partof other shapes in the box, so we
%go down the list and intersect the shapes with the convex cell

Obstacle_coords = [];
Obstacle_List_intersected = Obstacle_List_intersected;
for i = 1:length(Obstacle_List)
    Obstacle_List_intersected(i) = intersect(Obstacle_List_intersected(i),box,'KeepCollinearPoints',true);
    curver = Obstacle_List_intersected(i).Vertices;
    heighty = size(curver,1);
    inds = ones(heighty,1).*i;
    Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
end



%now y and go up
Obstacle_coords_rot = [Obstacle_coords(:,1:2)*R,Obstacle_coords(:,3)];
Obstacle_coords_rot_sorting = [Obstacle_coords_rot;[point1_rot,0];[point2_rot,0]];

%sort by y coordinate
[~,sortedInds] = sort(Obstacle_coords_rot_sorting(:,2));
Obstacle_coords_rot_sorting = Obstacle_coords_rot_sorting(sortedInds,:);
inds = find(Obstacle_coords_rot_sorting(:,3)==0);
myInd = inds(2); %first one is lowest, 2nd is highest 
incidence = find(Obstacle_coords_rot_sorting(:,2)==Obstacle_coords_rot_sorting(myInd,2)&...
    Obstacle_coords_rot_sorting(:,3)~=0);
outerCornerFound = false;
if myInd == length(Obstacle_coords_rot_sorting(:,3))
        %no obstacles!
        outerCornerFound = true;
elseif ~isempty(incidence)
        outerCornerFound = true;
    probPoint = Obstacle_coords_rot_sorting(incidence(1),1:2);
    box_rotated_vert = box.Vertices*R;
        %intersect with our halfspace
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) xlimrot(2) xlimrot(2)],...
            [ylimrot(1) probPoint(2) probPoint(2) ylimrot(1)]));
                cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) xlim(2) xlim(2)],...
            [ylim(1) probPoint(2) probPoint(2) ylim(1)]));
        
        cutBox = cutBox.Vertices;
%         cutBox = cutpolygon(box_rotated_vert,[[min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))]',[probPoint(2),probPoint(2)]'],...
%             2,true,false,false);
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
else
    guess = myInd+1;
end
while ~outerCornerFound
    %there is something below the lower point
    obstacleInGuess = Obstacle_coords_rot_sorting(guess,3);
    %find y coordinates of all points for obstacle in guess
    indsOfGuess = find(Obstacle_coords_rot_sorting(:,3) == obstacleInGuess);
    if all(indsOfGuess<=guess)
        %these y values all less than our guess point so we found the
        %end of the obstacle, this point is not a valid constraint
        guess = guess+1;
        if guess == length(Obstacle_coords_rot_sorting(:,3))+1
            outerCornerFound = true; %no restraint
        elseif Obstacle_coords_rot_sorting(guess,3) == 0
            guess = guess+1;
        end
    else
        %we know what point is the problem
        outerCornerFound = true;
        %we need to rotate the box into our frame
        probPoint = Obstacle_coords_rot_sorting(guess,1:2);
        box_rotated_vert = box.Vertices*R;
        %intersect with our halfspace
        xlimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        ylimrot = [min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))];
        cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlimrot(1) xlimrot(1) xlimrot(2) xlimrot(2)],...
            [ylimrot(1) probPoint(2) probPoint(2) ylimrot(1)]));
                cutBox = intersect(polyshape(box_rotated_vert(:,1),box_rotated_vert(:,2)),...
            polyshape([xlim(1) xlim(1) xlim(2) xlim(2)],...
            [ylim(1) probPoint(2) probPoint(2) ylim(1)]));
        
        cutBox = cutBox.Vertices;
%         cutBox = cutpolygon(box_rotated_vert,[[min(box_rotated_vert(:,1)),max(box_rotated_vert(:,1))]',[probPoint(2),probPoint(2)]'],...
%             2,true,false,false);
        %cut box is a polygon in rotated coordinates carrying the safe set
        cutBox_ogframe = cutBox*R_back;
        box = intersect(box,polyshape(cutBox_ogframe(:,1),cutBox_ogframe(:,2)));
    end
end

%now we want to only care about the partof other shapes in the box, so we
%go down the list and intersect the shapes with the convex cell

% Obstacle_coords = [];
% Obstacle_List_intersected = Obstacle_List_intersected;
% for i = 1:length(Obstacle_List)
%     Obstacle_List_intersected(i) = intersect(Obstacle_List_intersected(i),box,'KeepCollinearPoints',true);
%     curver = Obstacle_List_intersected(i).Vertices;
%     heighty = size(curver,1);
%     inds = ones(heighty,1).*i;
%     Obstacle_coords(end+1:end+heighty,:) = [curver,inds];
% end
% 


end
