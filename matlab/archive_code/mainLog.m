% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script logs all the necessary data for offline analysis. It will calibrate the anchor positions 
% and record range measurements from the anchors as well as ground-truth positions from VICON.

clear;
clc;

disp("This script logs all the necessary data for offline analysis. It will calibrate the anchor positions");
disp("and record range measurements from the anchors as well as ground-truth positions from VICON.");
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
ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
pause(5); % time needed for initialization

%% Testing Vicon (only for testing purpose, need to be turned off)

% GroundTruthIterationIndex = 1;
% 
% while true
% [DronePositionGroundTruthArray(1:3,GroundTruthIterationIndex),DroneQuaternionGroundTruthArray(1:4,GroundTruthIterationIndex)] = getGroundTruth(ViconDroneSubscriber); % gather ground-truth data
% GroundTruthIterationIndex = GroundTruthIterationIndex + 1;
% end

%% Set desired parameters

NumberOfAnchors = 6;
NumberOfIterations = 1000; % amount of range batches to be gathered
NumberOfIterationsForCalibration = 100; % amount of ranges to be gathered before averaged
TagMarker = [-40.4669,6.09042,68.6882];

%% Calling anchor calibration executables

input("Place anchors in the room and press [ENTER]");
input("Change the connected module into Sniffer mode and press [ENTER]");
disp("******************************************************************************************************");
disp("Starting anchor self-calibration procedure");

% starting anchor self-calibration procedure
AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors);
AnchorPositions = AnchorCalibration(AnchorRangeMean,NumberOfAnchors);

%% Calling the log script

disp("******************************************************************************************************");
disp("Preparing to gather range measurements");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
disp("******************************************************************************************************");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(SerialObject);
clear serial;
PortAddress = seriallist;
SerialObject = serial(PortAddress);

[TimeArray,RangeArray,DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray] = logRangeMeasurement(SerialObject,ViconDroneSubscriber,NumberOfIterations,NumberOfAnchors); % starting logging time, range and ground-truth measurement
save('rangemeasurement.mat','AnchorPositions','TimeArray','RangeArray','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','TagMarker'); % saving to workspace

%% Closing ROS communication

disp("Finished gathering data, shutting the system down");
rosshutdown();

