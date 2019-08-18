% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This script logs data for orientation dependency evaluation.

clear;
clc;

disp("This script logs data for orientation dependency evaluation.");
disp("******************************************************************************************************");

%% Setup serial port and ROS communication

logVicon = false;

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

ViconAnchorSubscriber = [];
ViconDroneSubscriber = [];

TagMarker = [-20.7756;34.7541;87.6537]/1000;
AnchorMarker = [22.6417;3.00382;87.7027]/1000;

if logVicon == true
    rosinit;
    RosTopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
    RosTopicAnchors = '/vicon/Anchors_Johann/Anchors_Johann';
    ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
    ViconAnchorsSubscriber = rossubscriber(RosTopicAnchors);  % subscriber object for anchors network
    pause(1); % time needed for initialization
end

%% Set desired parameters

NumberOfIterations = 1000;

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

AnchorRotationAngle = atan2(2*(AnchorsQuaternionGroundTruth(1)*AnchorsQuaternionGroundTruth(4)+AnchorsQuaternionGroundTruth(2)*AnchorsQuaternionGroundTruth(3)),(1-2*(AnchorsQuaternionGroundTruth(3)^2+AnchorsQuaternionGroundTruth(4)^2)));
AnchorRotationAngle = AnchorRotationAngle/(2*pi)*360;

figure()
plot(TimeArray,RangeArray,'b.','MarkerSize',5); % plotting
hold on

mean(RangeArray)

%% Saving

save('Rangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray','AnchorRotationAngle','ActualDistance'); % saving to workspace

%% Data postprocessing for rotation

ErrorArray = 2-RangeArray/1000; % calculating error offset

% getting z-rotation and mapping to degree
for i = 1:size(ErrorArray,2)
    RotationAngles(i) = atan2(2*(DroneQuaternionGroundTruthArray(1,i)*DroneQuaternionGroundTruthArray(4,i)+DroneQuaternionGroundTruthArray(2,i)*DroneQuaternionGroundTruthArray(3,i)),(1-2*(DroneQuaternionGroundTruthArray(3,i)^2+DroneQuaternionGroundTruthArray(4,i)^2)));
    RotationAngles(i) = RotationAngles(i)/(2*pi)*360;
end

AnchorRotationAngle = atan2(2*(AnchorsQuaternionGroundTruth(1)*AnchorsQuaternionGroundTruth(4)+AnchorsQuaternionGroundTruth(2)*AnchorsQuaternionGroundTruth(3)),(1-2*(AnchorsQuaternionGroundTruth(3)^2+AnchorsQuaternionGroundTruth(4)^2)));
AnchorRotationAngle = AnchorRotationAngle/(2*pi)*360;

figure()
plot(RotationAngles,ErrorArray,'ro','MarkerSize',2);
axis([-180 180 0 0.3])
hold on

%[SinusoidalCoefficients,SinusoidalFunction] = SinusoidalFit(RotationAngles,ErrorArray);
%plot(linspace(-180,180),SinusoidalFunction(SinusoidalCoefficients,linspace(-180,180)),'b--','LineWidth',2);
%grid on

%legend('Raw Range Measurements','Sinusoidal Fit');
%hold off

%% Saving

save('Rangemeasurement.mat','AnchorsPositionGroundTruth','AnchorsQuaternionGroundTruth','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','RangeArray','TimeArray','AnchorRotationAngle'); % saving to workspace
