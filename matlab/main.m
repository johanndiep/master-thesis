% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This program is the main function which is used to call each executable 
% for the Gauss-Newton position estimation method. It will guide the user 
% through each steps, starting from anchor setup self-calibration through 
% gathering waypoint datas on the Bebop drone with the UWB-ranging method 
% as well as with the VICON system. 

clear;
clc;

disp("This program is the main function which is used to call each executable ");
disp("for the Gauss-Newton position estimation method. It will guide the user");
disp("through each steps, starting from anchor setup self-calibration through");
disp("gathering waypoint datas on the Bebop drone with the UWB-ranging method");
disp("as well as with the VICON system.");
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

IterationIndex = 1;
HeightTop = 2.43-0.275; % top anchors heights measured from bottom anchor
AnchorMarkerDeviation = 0.03; % measured deviation of anchor markers to bottom anchor

% measured deviation of VICON-frame from world-frame, required the VICON frame to be placed at the corner of the origin anchor
X_ViconToWorld = 0.255;
Y_ViconToWorld = 0.215;
Z_ViconToWorld = -0.225;

% homogeneous transformation matrix, assumed to have no rotation
T_ViconToWorld = [1,0,0,X_ViconToWorld;0,1,0,Y_ViconToWorld;0,0,1,Z_ViconToWorld;0,0,0,1];

% read from VICON system, need to be adjusted each experiment
DroneMarkerBodyFrame = [-20.6485/1000;6.76277/1000;77.7595/1000];
FirstAnchorBodyFrame = [-1165.03/1000;-1641.83/1000;21.9812/1000];
SecondAnchorBodyFrame = [2353.66/1000;63.3209/1000;-34.1397/1000];
ThirdAnchorBodyFrame = [-1188.64/1000;1578.51/1000;12.1585/1000];

%% Set desired parameters

NumberOfAnchors = 6;
NumberOfIterations = 100; % number of position data
NumberOfIterationsForCalibration = 100; % amount of ranges to be gathered before averaged for anchor calibration
NumberOfIterationsForRanging = 10; % amount of ranges to be gathered before averaged for ranging

%% Calling anchor calibration executables

input("Place anchors in the room and press [ENTER]");
input("Change the connected module into Sniffer mode and press [ENTER]");
disp("******************************************************************************************************");
disp("Starting anchor self-calibration procedure");

% starting anchor self-calibration procedure
AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors);
AnchorPositions = AnchorCalibration(AnchorRangeMean,NumberOfAnchors);

%% Plotting the anchors

figure()
hold on
title("Tinamu Labs Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
grid on;

scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);

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

for i = 1:size(AnchorPositions,1)
    text(AnchorPositions(i,1)+0.1,AnchorPositions(i,2)+0.1,AnchorPositions(i,3)+0.1,"Anchor "+int2str(i));
end

% ground-truth anchors network coordingt_arrayate transformation
[AnchorsPositionGroundTruth,~] = getAnchorsGroundTruth(ViconAnchorsSubscriber);
FirstAnchorWorldFrame = T_ViconToWorld * [eye(3),AnchorsPositionGroundTruth;0,0,0,1] * [FirstAnchorBodyFrame;1];
SecondAnchorWorldFrame = T_ViconToWorld * [eye(3),AnchorsPositionGroundTruth;0,0,0,1] * [SecondAnchorBodyFrame;1];
ThirdAnchorWorldFrame = T_ViconToWorld * [eye(3),AnchorsPositionGroundTruth;0,0,0,1] * [ThirdAnchorBodyFrame;1];

% plotting anchors network
AnchorsWorldFrame = [FirstAnchorWorldFrame,SecondAnchorWorldFrame,ThirdAnchorWorldFrame];
for i = 1:size(AnchorsWorldFrame,2)
    scatter3(AnchorsWorldFrame(1,i),AnchorsWorldFrame(2,i),AnchorsWorldFrame(3,i)-AnchorMarkerDeviation,'MarkerEdgeColor','y','MarkerFaceColor',[1,1,0]);
    scatter3(AnchorsWorldFrame(1,i),AnchorsWorldFrame(2,i),AnchorsWorldFrame(3,i)+(HeightTop-AnchorMarkerDeviation),'MarkerEdgeColor','y','MarkerFaceColor',[1,1,0]);
end

%% Calling the position estimation executables

disp("******************************************************************************************************");
disp("Preparing to gather " + NumberOfIterations + " waypoints");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
disp("******************************************************************************************************");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(SerialObject);
clear serial;
PortAddress = seriallist;
SerialObject = serial(PortAddress);

% plotting starting position
RangeMean = getRangeMeasurement(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors);
CurrentTagPosition = TagPositionEstimation(AnchorPositions,RangeMean,NumberOfAnchors);
scatter3(CurrentTagPosition(1),CurrentTagPosition(2),CurrentTagPosition(3),5,'r');

% plotting starting ground-truth position
[CurrentDronePositionGroundTruth,CurrentDroneQuaternionGroundTruth] = getGroundTruth(ViconDroneSubscriber);
CurrentDroneRotationGroundTruth = quat2rotm(CurrentDroneQuaternionGroundTruth');
CurrentDroneMarkerPositionGroundTruth = T_ViconToWorld * [CurrentDroneRotationGroundTruth,CurrentDronePositionGroundTruth;0,0,0,1] * [DroneMarkerBodyFrame;1];
scatter3(CurrentDroneMarkerPositionGroundTruth(1),CurrentDroneMarkerPositionGroundTruth(2),CurrentDroneMarkerPositionGroundTruth(3),5,'b');

while IterationIndex < NumberOfIterations + 1
    disp("Position number " + IterationIndex + " of " + NumberOfIterations);
    
    % estimating tag position
    tic;
    RangeMean = getRangeMeasurement(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors);
    NextTagPosition = TagPositionEstimation(AnchorPositions,RangeMean,NumberOfAnchors);
    Time = toc;
    disp("- New position estimated in " + Time + " seconds");
    disp("- UWB position estimation frequency: " + 1/Time + " Hz");
    
    % reading ground-truth position
    tic;
    [NextDronePositionGroundTruth,NextDroneQuaternionGroundTruth] = getGroundTruth(ViconDroneSubscriber);
    NextDroneRotationGroundTruth = quat2rotm(NextDroneQuaternionGroundTruth');
    NextDroneMarkerPositionGroundTruth = T_ViconToWorld * [NextDroneRotationGroundTruth,NextDronePositionGroundTruth;0,0,0,1] * [DroneMarkerBodyFrame;1];
    Time = toc;
    disp("- New ground-truth position aquired in " + Time + " seconds");
    disp("- VICON position estimation frequency: " + 1/Time + " Hz");
    disp("******************************************************************************************************");
    
    % plotting the estimated and ground-truth positions
    scatter3(NextTagPosition(1),NextTagPosition(2),NextTagPosition(3),5,'r');
    scatter3(NextDroneMarkerPositionGroundTruth(1),NextDroneMarkerPositionGroundTruth(2),NextDroneMarkerPositionGroundTruth(3),5,'b');
    
    % connecting neighboring positions
    line([CurrentTagPosition(1),NextTagPosition(1)],[CurrentTagPosition(2),NextTagPosition(2)],[CurrentTagPosition(3),NextTagPosition(3)],'Color',[1,.6196,.6196]);
    line([CurrentDroneMarkerPositionGroundTruth(1),NextDroneMarkerPositionGroundTruth(1)],[CurrentDroneMarkerPositionGroundTruth(2),NextDroneMarkerPositionGroundTruth(2)],[CurrentDroneMarkerPositionGroundTruth(3),NextDroneMarkerPositionGroundTruth(3)],'Color',[.6196,.6196,1]);
    drawnow;
    
    % update
    CurrentTagPosition = NextTagPosition;
    CurrentDroneMarkerPositionGroundTruth = NextDroneMarkerPositionGroundTruth;
    IterationIndex = IterationIndex + 1;
end

%% Closing ROS communication

disp("Finished gathering waypoints, shutting the system down");
rosshutdown;
