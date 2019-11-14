clear; clc;

rosinit;
AnchorSub = rossubscriber('/vicon/Anchors_Johann/Anchors_Johann');

%%

[VicAnchorPos,VicAnchorQuat] = getGroundTruth(AnchorSub);

MarkP1 = [-1358.65,-2164.78,20.9967]/1000;
MarkP2 = [2688.71,679.85,-44.905]/1000;
MarkP3 = [-1330.07,1484.93,23.9083]/1000;

Ta = diag(ones(1,4));
Ta(1:3,1:3) = quat2rotm(VicAnchorQuat');
Ta(1:3,4) = VicAnchorPos;

Dev(1) = 4/100;
Dev(2) = 2.16-Dev(1);

A(:,1) = Ta*[MarkP1'+[0;0;-Dev(1)];1];
A(:,2) = Ta*[MarkP1'+[0;0;Dev(2)];1];
A(:,3) = Ta*[MarkP2'+[0;0;-Dev(1)];1];
A(:,4) = Ta*[MarkP2'+[0;0;Dev(2)];1];
A(:,5) = Ta*[MarkP3'+[0;0;-Dev(1)];1];
A(:,6) = Ta*[MarkP3'+[0;0;Dev(2)];1];
A(4,:) = [];

save('AnchorPosGroundTruth.mat','A','VicAnchorPos','VicAnchorQuat');

rosshutdown;

%%

load('AnchorPosGroundTruth.mat');
load('AnchorPos.mat');

T = diag(ones(1,4));
T(1:3,4) = [-0.23;-0.25;0.25];
An = T*[AnchorPos';ones(1,6)]; AnchorPos = A(1:3,:)';

figure()
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([-1,5]);
ylim([-1,4]);
zlim([0,2.5]);
hold on;

scatter3(A(1,:),A(2,:),A(3,:),30,'ro');
hold on;
scatter3(An(1,:),An(2,:),An(3,:),100,'r+');

set(0,'DefaultLegendAutoUpdate','off')
legend('Ground-Truth','Calculated Anchor Positions');

for i = 1:size(A,2)
    text(A(1,i)+0.1,A(2,i)+0.1,A(3,i)+0.1, ...
        "A"+int2str(i));
end

grid on;
hold off;
