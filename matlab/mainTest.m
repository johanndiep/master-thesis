% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script is for offline parameter tuning and analysis.

%% Plotting the anchors

% figure()
% hold on
% title("Tinamu Labs Flying Machine Arena");
% xlabel("x-Axis [m]");
% ylabel("y-Axis [m]");
% zlabel("z-Axis [m]");
% grid on

%scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);

NumberOfAnchors = 6;

if NumberOfAnchors == 8
    AnchorCombinations = [1,5;4,8;2,6;3,7];
    for i = 1:size(AnchorCombinations,1)
        line([AnchorPositions(AnchorCombinations(i,1),1),AnchorPositions(AnchorCombinations(i,2),1)],[AnchorPositions(AnchorCombinations(i,1),2),AnchorPositions(AnchorCombinations(i,2),2)],[AnchorPositions(AnchorCombinations(i,1),3),AnchorPositions(AnchorCombinations(i,2),3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
    end
elseif NumberOfAnchors == 6
    for i = 1:size(AnchorPositions,1)
        if mod(i,2) == 1
            line([AnchorPositions(i,1),AnchorPositions(i+1,1)],[AnchorPositions(i,2),AnchorPositions(i+1,2)],[AnchorPositions(i,3),AnchorPositions(i+1,3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
        end
    end
end

% for i = 1:size(AnchorPositions,1)
%     text(AnchorPositions(i,1)+0.1,AnchorPositions(i,2)+0.1,AnchorPositions(i,3)+0.1,"Anchor "+int2str(i));
% end

%% Ground-truth

index = 1;

X_ViconToWorld = 0.23;
Y_ViconToWorld = 0.245;
Z_ViconToWorld = -0.23;

T_ViconToWorld = [1,0,0,X_ViconToWorld; ...
    0,1,0,Y_ViconToWorld; ...
    0,0,1,Z_ViconToWorld;
    0,0,0,1];

tag_BodyFrame = TagMarker'/1000;

for i = 1:size(DronePositionGroundTruthArray,2)
    A = quat2rotm(DroneQuaternionGroundTruthArray(:,i)');
    b = DronePositionGroundTruthArray(:,i);
    c(:,index) = T_ViconToWorld * [A,b;0,0,0,1] * [tag_BodyFrame;1];
    index = index + 1;
end

%scatter3(c(1,:),c(2,:),c(3,:),0.5,"b");

%% Estimation Gauss-Newton

starting_position = TagPositionEstimation(AnchorPositions,RangeArray(:,1),NumberOfAnchors);
%scatter3(starting_position(1),starting_position(2),starting_position(3),5,"r");

%% Estimation Kalman-Filter

PreviousTime = 0;

x_posterior = [starting_position,normrnd(0,0.1,[1,3])]';
P_posterior = 0.05*eye(size(x_posterior,1));

SavedWaypoints(1,1:3) = x_posterior(1:3);

[h,H] = PreprocessingVanillaEKF(AnchorPositions,0);

for i = 2:size(RangeArray,2)
   z = RangeArray(:,i)/1000;
   TimeSinceStart = TimeArray(6,i);
   [x_posterior,P_posterior] = VanillaEKF(6,x_posterior,P_posterior,TimeSinceStart-PreviousTime,z,h,H,AnchorPositions);
   SavedWaypoints(i,1:3) = x_posterior(1:3);
   PreviousTime = TimeSinceStart;
end

%%
% scatter3(SavedWaypoints(:,1),SavedWaypoints(:,2),SavedWaypoints(:,3),5,"r")

c_truncated = c(:,1:6:end-1);

figure();
hold on;
grid on
for i = 1:1:1000
    c_truncated(1:3,i) = c_truncated(1:3,i)/norm(c_truncated(1:3,i))*Offset(1,i);
    scatter3(c_truncated(1,i),c_truncated(2,i),c_truncated(3,i),'bo');
end

%%

for i = 1:6001
    for j = 1:6
       RealRanges(j,i) = sqrt((c(1,i)-AnchorPositions(j,1))^2 + (c(2,i)-AnchorPositions(j,2))^2 + (c(3,i)-AnchorPositions(j,3))^2);
    end
end

RealRangesTruncated = RealRanges(:,1:6:end-1);

Offset = RealRangesTruncated - RangeArray/1000;