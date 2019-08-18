% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This program is the main function which is used to call each executable for the EKF method.
% It will guide the user through each steps, starting from anchor setup self-calibration
% through gathering waypoint datas on the Bebop drone with the UWB-ranging method.

clear;
clc;

disp("This program is the main function which is used to call each executable for the EKF method.");
disp("It will guide the user through each steps, starting from anchor setup self-calibration");
disp("through gathering waypoint datas on the Bebop drone with the UWB-ranging method.");
disp("******************************************************************************************************");

%% Setup serial port

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

%% Hardcoded parameters

IterationIndex = 1;
PreviousTime = 0;

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
grid on

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

%% Calling the position estimation executables

("******************************************************************************************************");
disp("Preparing to gather " + NumberOfIterations + " waypoints");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
("******************************************************************************************************");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(SerialObject);
clear serial;
PortAddress = seriallist;
SerialObject = serial(PortAddress);

% using Gauss-Newton for Kalman-Filter initialization 
RangeMean = getRangeMeasurement(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors);
StartingPosition = TagPositionEstimation(AnchorPositions,RangeMean,NumberOfAnchors);

% initialization of state [p_x,p_y,p_z,v_x,v_y,v_z] and covariance
x_Posterior = [StartingPosition,normrnd(0,0.1,[1,3])]';
P_Posterior = 0.05*eye(size(x_Posterior,1));
SavedWaypoints(1,1:3) = x_Posterior(1:3);

[h,H] = PreprocessingVanillaEKF(AnchorPositions); % preprocessing

tic; % starting timer
while IterationIndex < NumberOfIterations + 1
    z = getRangeMeasurement(SerialObject,1,NumberOfAnchors)'/1000; % getting range measurements for single batch
    
    TimeSinceStart = toc;
    DeltaT = TimeSinceStart-PreviousTime;
    [x_Posterior,P_Posterior] = VanillaEKF(NumberOfAnchors,x_Posterior,P_Posterior,DeltaT,z,h,H); % estimating a posteriori position
    SavedWaypoints(IterationIndex+1,1:3) = x_Posterior(1:3);
    
    % update
    IterationIndex = IterationIndex + 1;
    PreviousTime = TimeSinceStart;
end

%% Plotting
 
scatter3(SavedWaypoints(:,1),SavedWaypoints(:,2),SavedWaypoints(:,3),5,'r'); % plotting points
    
% connecting neighboring positions
for i = 1:(size(SavedWaypoints,1)-1)
    line([SavedWaypoints(i,1),SavedWaypoints(i+1,1)],[SavedWaypoints(i,2),SavedWaypoints(i+1,2)],[SavedWaypoints(i,3),SavedWaypoints(i+1,3)],'Color',[1,.6196,.6196]);
end
