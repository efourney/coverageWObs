thet = 0:2:360;
x1 = 1.*cosd(thet);
y1 = zeros(1,length(x1));
z1 = 1.*sind(thet);

cent1 = [x1;y1;z1];
cent2 = rotz(90)*cent1;
cent3 = rotz(90)*cent1;
cent4 = rotx(90)*cent1;

[X,Y,Z] = sphere;

figure
hold on
for i = 1:length(cent1)
    surf(X + cent1(1,i),Y+cent1(2,i),Z+cent1(3,i))
end

for i = 1:length(cent2)
    surf(X + cent2(1,i),Y+cent2(2,i),Z+cent2(3,i))
end

for i = 1:length(cent3)
    surf(X + cent3(1,i),Y+cent3(2,i),Z+cent3(3,i))
end

for i = 1:length(cent4)
    surf(X + cent4(1,i),Y+cent4(2,i),Z+cent4(3,i))
end

surf(2.*X,2.*Y,2.*Z)
