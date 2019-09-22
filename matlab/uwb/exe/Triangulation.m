% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% This script triangulates the position from 6 range measurements.

clear; clc;

load('GPCircConData.mat');
load('HyperparametersGP.mat');

%% Triangulation

for i = 1:size(Savet,2)
   ObjNorm = @(p) getTriangulationNorm(Savet(:,i),AnchorPos,p);
   Tp(:,i) = fmincon(ObjNorm,[0;0;0],[],[],[],[],[],[],[]);
end

for i = 1:size(SaveAbs,2)
   ObjNorm = @(s) getTriangulationNorm(SaveAbs(:,i),AnchorPos,s);
   P(:,i) = fmincon(ObjNorm,[0;0;0],[],[],[],[],[],[],[]);    
end

for i = 1:size(SaveRangeArr,2)
   ObjNorm = @(u) getTriangulationNorm(SaveRangeArr(:,i),AnchorPos,u);
   S(:,i) = fmincon(ObjNorm,[0;0;0],[],[],[],[],[],[],[]);    
end

%% Plotting

figure();
title("Triangulation");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([-2,2]);
ylim([-2,2]);
hold on;

scatter3(Tp(1,:),Tp(2,:),Tp(3,:),'k.');
scatter3(P(1,:),P(2,:),P(3,:),'r.');
scatter3(S(1,:),S(2,:),S(3,:),'b.');
scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:),'g.');

set(0,'DefaultLegendAutoUpdate','off')
legend('Corrected Ranging Model','Conventional Ranging Model','True Ranges', ...
    'Ground-Truth');

grid on;
hold off;