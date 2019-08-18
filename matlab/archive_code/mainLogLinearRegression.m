% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script logs data for linear regression evaluation.

clear;
clc;

disp("This script logs data for linear regression evaluation.");
disp("******************************************************************************************************");

%% Setup serial port and ROS communication

logVicon = true;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

ViconAnchorSubscriber = [];
ViconDroneSubscriber = [];

if logVicon == true
    rosinit;
    RosTopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
    RosTopicAnchors = '/vicon/Anchors_Johann/Anchors_Johann';
    ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
    ViconAnchorsSubscriber = rossubscriber(RosTopicAnchors);  % subscriber object for anchors network
    pause(1); % time needed for initialization
end

%% Set desired parameters

NumberOfIterations = 5000;

%% Data gathering

if logVicon == true
    [AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth] = getAnchorsGroundTruth(ViconAnchorsSubscriber); 
end
[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations,logVicon);

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

%% Saving

save('LRRangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray'); % saving to workspace

%% Linear Regression

TagMarker = [-20.7756;34.7541;87.6537]/1000;
AnchorMarker = [22.6417;3.00382;87.7027]/1000;

AnchorViconFrame(1:4,1) = getCoordinateViconFrame(AnchorsQuaternionGroundTruth,AnchorsPositionGroundTruth,AnchorMarker);
AnchorViconFrame(4) = [];

figure()
hold on
scatter3(AnchorViconFrame(1),AnchorViconFrame(2),AnchorViconFrame(3),'b');

for i = 1:size(DronePositionGroundTruthArray,2)
    TagViconFrame(1:4,i) = getCoordinateViconFrame(DroneQuaternionGroundTruthArray(:,i),DronePositionGroundTruthArray(:,i),TagMarker);
    ActualDistance(i) = norm(TagViconFrame(1:3,i)-AnchorViconFrame);
    DroneRotationAngle(i) = atan2(2*(DroneQuaternionGroundTruthArray(1,i)*DroneQuaternionGroundTruthArray(4,i)+DroneQuaternionGroundTruthArray(2,i)*DroneQuaternionGroundTruthArray(3,i)),(1-2*(DroneQuaternionGroundTruthArray(3,i)^2+DroneQuaternionGroundTruthArray(4,i)^2)));
    DroneRotationAngle(i) = DroneRotationAngle(i)/(2*pi)*360;
end
scatter3(TagViconFrame(1,:)',TagViconFrame(2,:)',TagViconFrame(3,:)','r');

AnchorRotationAngle = atan2(2*(AnchorsQuaternionGroundTruth(1)*AnchorsQuaternionGroundTruth(4)+AnchorsQuaternionGroundTruth(2)*AnchorsQuaternionGroundTruth(3)),(1-2*(AnchorsQuaternionGroundTruth(3)^2+AnchorsQuaternionGroundTruth(4)^2)));
AnchorRotationAngle = AnchorRotationAngle/(2*pi)*360;

figure()
plot(ActualDistance,RangeArray/1000,'b.','MarkerSize',5);
hold on
plot(ActualDistance,ActualDistance,'r.','MarkerSize',2);
axis([0.5 3.4 0 3.4])