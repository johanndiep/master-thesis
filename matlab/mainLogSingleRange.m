% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script logs data for orientation dependency evaluation.

clear;
clc;

disp("This script logs data for orientation dependency evaluation.");
disp("******************************************************************************************************");

%% Setup serial port and ROS communication

logVicon = true;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

ViconDroneSubscriber = [];
ViconAnchorSubscriber = [];

TagMarker = [1;1;1];
AnchorMarker = [2;2;2];

if logVicon == true
    rosinit;
    RosTopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
    RosTopicAnchors = '/vicon/Anchors_Johann/Anchors_Johann';
    ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
    ViconAnchorsSubscriber = rossubscriber(RosTopicAnchors);  % subscriber object for anchors network
    pause(5); % time needed for initialization
end

%% Set desired parameters

NumberOfIterations = 3000;

%% Data gathering

if logVicon == true
    [AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth] = getAnchorsGroundTruth(ViconAnchorsSubscriber); 
end
[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations,logVicon);

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

%% Data postprocessing for translation

for i = 1:size(DronePositionGroundTruthArray,2)
    TagViconFrame(1:4,i) = getCoordinateViconFrame(DroneQuaternionGroundTruthArray(:,i),DronePositionGroundTruthArray(:,i),TagMarker);
end
AnchorViconFrame(1:4,1) = getCoordinateViconFrame(AnchorsQuaternionGroundTruth,AnchorsPositionGroundTruth,AnchorMarker);
AnchorViconFrame(4) = [];

for i = 1:size(TagViconFrame,1)
    TagViconFrameMean(i,1) = mean(TagViconFrame(i,:));
end
TagViconFrameMean(4) = [];

ActualDistance = norm(TagViconFrameMean-AnchorViconFrame);

figure()
plot(TimeArray,RangeArray,'b.','MarkerSize',5); % plotting
hold on

%% Data postprocessing for rotation

ErrorArray = 1.5-RangeArray/1000; % calculating error offset

% getting z-rotation and mapping to degree
for i = 1:size(ErrorArray,2)
    RotationAngles(i) = atan2(2*(DroneQuaternionGroundTruthArray(1,i)*DroneQuaternionGroundTruthArray(4,i)+DroneQuaternionGroundTruthArray(2,i)*DroneQuaternionGroundTruthArray(3,i)),(1-2*(DroneQuaternionGroundTruthArray(3,i)^2+DroneQuaternionGroundTruthArray(4,i)^2)));
    RotationAngles(i) = RotationAngles(i)/(2*pi)*360;
end

AnchorRotationAngle = atan2(2*(AnchorsQuaternionGroundTruth(1)*AnchorsQuaternionGroundTruth(4)+AnchorsQuaternionGroundTruth(2)*AnchorsQuaternionGroundTruth(3)),(1-2*(AnchorsQuaternionGroundTruth(3)^2+AnchorsQuaternionGroundTruth(4)^2)));
AnchorRotationAngle = AnchorRotationAngle/(2*pi)*360;

figure()
plot(RotationAngles,ErrorArray,'b.','MarkerSize',5); % plotting
axis([-180 180 0 0.3])
hold on

%% Saving

save('GPrangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray','AnchorRotationAngle'); % saving to workspace
