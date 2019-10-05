clear; clc;

s = [0 1 2 3 4 5];
points = [0 0; 1 0; 1 1; 0 1; 0 2; 1 2];
x = points(:,1);
y = points(:,2);

tq = 0:0.1:5;
slope0 = 0;
slopeF = 0;
xq = spline(s,[slope0; x; slopeF],tq);
yq = spline(s,[slope0; y; slopeF],tq);

figure;
plot(x,y,'o',xq,yq,':.');
axis([-0.5 1.5 -0.5 2.5]);
title('y vs. x');

xqi = xq;
yqi = yq;

velx = circshift(xqi,[0,-1])-xqi;
vely = circshift(yqi,[0,-1])-yqi;
norm = vecnorm([velx;vely]);
