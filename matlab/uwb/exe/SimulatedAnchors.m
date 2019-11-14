clear; clc;

h = 2.2;
s = zeros(1,3);

AnchorPos = [0,0,0;0,0,h;5,4,0;5,4,h;0,6,0;0,6,h];

for i = 1:6
    for j = 1:6
        AnchorRangeMean(i,j) = norm(AnchorPos(i,:)-AnchorPos(j,:));
    end
end

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% objective function
ObjNorm = @(p) getObjectiveNorm(AnchorRangeMean,h,p(1),p(2),p(3));

p = fmincon(ObjNorm,s,[],[],[],[],[0,0,0],[6,6,6],[],options);

AnchorPosCal = [0,0,0;0,0,h;p(1),p(2),0;p(1),p(2),h;0,p(3),0;0,p(3),h];

figure()
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([-1,6]);
ylim([-1,7]);
zlim([0,2.5]);
hold on;

scatter3(AnchorPos(:,1),AnchorPos(:,2),AnchorPos(:,3),30,'ro');
scatter3(AnchorPosCal(:,1),AnchorPosCal(:,2),AnchorPosCal(:,3),100,'r+');

set(0,'DefaultLegendAutoUpdate','off')
legend('Ground-Truth','Calculated Anchor Positions');

for i = 1:size(AnchorPos,1)
    text(AnchorPos(i,1)+0.1,AnchorPos(i,2)+0.1,AnchorPos(i,3)+0.1, ...
        "A"+int2str(i));
end

grid on;
hold off;
