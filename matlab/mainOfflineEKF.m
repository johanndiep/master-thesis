% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script runs the offline EKF position estimation.

%% Execution
close all
clear all
load('rangemeasurement.mat')
PreviousTime = 0;

figure()
hold on
title("Tinamu Labs Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
grid on

scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),'bo');
scatter3(AnchorGroundTruthPositions(1,:),AnchorGroundTruthPositions(2,:),AnchorGroundTruthPositions(3,:),'ro');

scatter3(DroneGroundTruthPositions(1,:),DroneGroundTruthPositions(2,:),DroneGroundTruthPositions(3,:),'r.');

StartingPosition = DroneGroundTruthPositions(:,1);
x_Posterior = [StartingPosition',0,0,0]';
P_Posterior = 0.01^2*eye(size(x_Posterior,1));
SavedWaypoints(1,1:3) = x_Posterior(1:3);
[h,H] = PreprocessingVanillaEKF(AnchorGroundTruthPositions',[]);

Index = 1;
for i = 1:6:size(DroneGroundTruthPositions,2)
    for j = 1:6
        GroundTruthRanges(j,Index) = norm(AnchorGroundTruthPositions(:,j)-DroneGroundTruthPositions(:,i));
    end
    Index = Index + 1;
end

for i = 1:size(RangeArray,2)
   z = RangeArray(:,i)/1000;
   TimeSinceStart = TimeArray(6,i);
   [x_Posterior,P_Posterior] = VanillaEKF(6,x_Posterior,P_Posterior,TimeSinceStart-PreviousTime,z,h,H,AnchorGroundTruthPositions');
   SavedWaypoints(i,1:3) = x_Posterior(1:3);
   PreviousTime = TimeSinceStart;
end

scatter3(SavedWaypoints(:,1),SavedWaypoints(:,2),SavedWaypoints(:,3),'b+');
