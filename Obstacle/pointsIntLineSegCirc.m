function [points] = pointsIntLineSegCirc(point1,point2,circC,r)
%return boundary points 
%endpoints if endpoint is in circle
%intersection with circle
points = [];
%check if we have an endpoint in the circle, account for minor rounding
%errors
if norm(point1-circC)<=r+eps
    points = point1;
end
if norm(point2-circC)<=r+eps
    points(end+1,:) = point2;
end
%now solve for where line segment intersecgts circle
ax = point1(1);
ay = point1(2);
bx = point2(1);
by = point2(2);
cx = circC(1);
cy = circC(2);
px = bx-ax;
py = by-ay;
qx = ax-cx;
qy = ay-cy;
a = px^2+py^2;%only zero if same point;
if a == 0
    return;
end
b = 2*(px*qx+py*qy);
c = qx^2+qy^2-r^2;

gam1 = (-b+sqrt(b^2-4*a*c))/(2*a);
gam2 = (-b-sqrt(b^2-4*a*c))/(2*a);

if abs(imag(gam1))<=eps*10000
    gam1 = real(gam1);
    if gam1<=1 && gam1>=0
        points(end+1,:) = point1 + gam1.*(point2-point1);
    end
end

if abs(imag(gam2))<=eps*10000 && abs(sqrt(b^2-4*a*c))>=eps*10^12
    gam2 = real(gam2);
    if gam2<=1 && gam2>=0
        points(end+1,:) = point1 + gam2.*(point2-point1);
    end
end

%we should clarify we have unique points, but should be fine
points = uniquetol(points,'ByRows',true);


end