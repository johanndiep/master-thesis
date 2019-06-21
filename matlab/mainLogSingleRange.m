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

NumberOfIterations = 10000;

%% Data gathering

[AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth] = getAnchorsGroundTruth(ViconAnchorsSubscriber); 
[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations);

save('GPrangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray'); % saving to workspace

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

%% Data postprocessing

ErrorArray = RangeArray/1000; % calculating error offset

% Shifting angles to avoid flip at 120 degree
RotationAngles = 2 * acos(DroneQuaternionGroundTruthArray(1,:))/(2*pi)*360;
CopyRotationAngles = RotationAngles;
ShiftAngle = false;
for i = 1+30:size(RotationAngles,2)-30
    if abs(CopyRotationAngles(i) - CopyRotationAngles(i-1)) > 100
        ShiftAngle = true;
    elseif CopyRotationAngles(i) == min(CopyRotationAngles(i-30:i+30)) && ShiftAngle
        ShiftAngle = false;
    end
    
    if ShiftAngle
        RotationAngles(i) = 360-CopyRotationAngles(i);
    end 
end

plot(RotationAngles,ErrorArray,'b.','MarkerSize',3); % plotting
hold on;