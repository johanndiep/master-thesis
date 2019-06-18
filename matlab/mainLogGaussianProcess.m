% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script logs data for Gaussian Process evaluation.

clear;
clc;

disp("This script logs data for Gaussian Process evaluation.");
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

%% Hardcoded parameters and coordinate transformations

AnchorMarkerDeviation = 0.035; % measured deviation of anchor markers to anchor

% read from VICON system, need to be adjusted each experiment
DroneMarkerBodyFrame = [-10.9356/1000;-16.7161/1000;81.1309/1000];
AnchorBodyFrame = [-363.413/1000;1118.1/1000;447.414/1000];

%% Set desired parameters

NumberOfIterations = 10000;

%% Data gathering

[AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth] = getAnchorsGroundTruth(ViconAnchorsSubscriber); 
[DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logGaussianProcessData(SerialObject,ViconDroneSubscriber,NumberOfIterations);

save('GPrangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray'); % saving to workspace

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

