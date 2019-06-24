% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script logs data for orientation dependency evaluation.

clear;
clc;

disp("This script logs data for orientation dependency evaluation.");
disp("******************************************************************************************************");

%% Setup serial port and ROS communication

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

rosinit;
RosTopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
RosTopicAnchors = '/vicon/Anchors_Johann/Anchors_Johann';
ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
ViconAnchorsSubscriber = rossubscriber(RosTopicAnchors);  % subscriber object for anchors network
pause(5); % time needed for initialization

%% Set desired parameters

NumberOfIterations = 3000;

%% Data gathering

[AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth] = getAnchorsGroundTruth(ViconAnchorsSubscriber); 
[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations);

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

%% Data postprocessing

ErrorArray = 2-RangeArray/1000; % calculating error offset

% getting z-rotation and mapping to degree
for i = 1:size(ErrorArray,2)
    RotationAngles(i) = atan2(2*(DroneQuaternionGroundTruthArray(1,i)*DroneQuaternionGroundTruthArray(4,i)+DroneQuaternionGroundTruthArray(2,i)*DroneQuaternionGroundTruthArray(3,i)),(1-2*(DroneQuaternionGroundTruthArray(3,i)^2+DroneQuaternionGroundTruthArray(4,i)^2)));
    RotationAngles(i) = RotationAngles(i)/(2*pi)*360;
end

AnchorRotationAngle = atan2(2*(AnchorsQuaternionGroundTruth(1)*AnchorsQuaternionGroundTruth(4)+AnchorsQuaternionGroundTruth(2)*AnchorsQuaternionGroundTruth(3)),(1-2*(AnchorsQuaternionGroundTruth(3)^2+AnchorsQuaternionGroundTruth(4)^2)));
AnchorRotationAngle = AnchorRotationAngle/(2*pi)*360;

plot(RotationAngles,ErrorArray,'bx','MarkerSize',3); % plotting
hold on;

%% Saving

save('GPrangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray','AnchorRotationAngle'); % saving to workspace
