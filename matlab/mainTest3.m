clear
clc

rosinit;
pause(10);
RostopicAnchor = '/vicon/Anchors_Johann/Anchors_Johann';
RostopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
ViconAnchorsSubscriber = rossubscriber(RostopicAnchor);
ViconDroneSubscriber = rossubscriber(RostopicDrone);

NumberOfIterations = 1000;

%%

[a,b]=getAnchorsGroundTruth(ViconAnchorsSubscriber);
c = getAnchorsPosition(6,a,b);

%%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

%%

[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations,true);

%%

MarkerBodyFrame = [-37.6483;4.75431;68.837]/1000;
for i = 1:size(DroneQuaternionGroundTruthArray,2)
    PositionViconFrame(:,i) = getCoordinateViconFrame(DroneQuaternionGroundTruthArray(:,i),DronePositionGroundTruthArray(:,i),MarkerBodyFrame);
end
PositionViconFrame(4,:) = [];

%%

rosshutdown;

%%

%scatter3(c(1,:),c(2,:),c(3,:),5,"ro");
%hold on;
%scatter3(PositionViconFrame(1,:),PositionViconFrame(2,:),PositionViconFrame(3,:),5,"bo");

%%

for i = 1:NumberOfIterations
    ActualRangeDifference(i) = sqrt((PositionViconFrame(1,i)-c(1,1))^2+(PositionViconFrame(2,i)-c(2,1))^2+(PositionViconFrame(3,i)-c(3,1))^2);
end

if ismember(0,RangeArray)
    disp("Zeros");
    Zeros = find(RangeArray == 0);
    RangeArray(Zeros) = [];
    PositionViconFrame(:,Zeros) = [];
end

mean(ActualRangeDifference)

Offset = ActualRangeDifference-RangeArray/1000;

for i = 1:NumberOfIterations
    Difference(:,i) = PositionViconFrame(:,i) - c(:,1); 
    Difference(:,i) = Difference(:,i)/norm(Difference(:,i))*Offset(i);
end

%scatter3(Difference(1,:),Difference(2,:),Difference(3,:),1,"r.");

mean(Offset)
