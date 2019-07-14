% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script is for offline parameter tuning and analysis.

clear;
clc;

disp("This script is for offline parameter tuning and analysis.");
disp("******************************************************************************************************");

%%

rosinit;
pause(1);
RostopicAnchor = '/vicon/Anchors_Johann/Anchors_Johann';
ViconAnchorsSubscriber = rossubscriber(RostopicAnchor);

%%

[a,b]=getAnchorsGroundTruth(ViconAnchorsSubscriber);
c = getAnchorsPosition(6,a,b);
scatter3(c(1,:),c(2,:),c(3,:),5,"bo");
hold on;
rosshutdown;

%%

for i = 1:6
    for j = 1:6
        GroundTruthAnchorRanges(i,j) = sqrt((c(1,i)-c(1,j))^2+(c(2,i)-c(2,j))^2+(c(3,i)-c(3,j))^2);
    end
end

%%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

NumberOfIterationsForCalibration = 100; % amount of ranges to be gathered before averaged

AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,6);
AnchorPositions = AnchorCalibration(AnchorRangeMean,6);
scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),6,"ro");

Offset = GroundTruthAnchorRanges - AnchorRangeMean/1000;

FirstRanges = AnchorRangeMean;
%%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,100,6);
AnchorPositions = AnchorCalibration(AnchorRangeMean,6);
scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),7,"go");

SecondRanges = AnchorRangeMean;

AnchorRangeMean = AnchorRangeMean + Offset*1000;
AnchorPositions = AnchorCalibration(AnchorRangeMean,6);
scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),8,"ko");

ThirdRanges = AnchorRangeMean;

legend('GT','First estimation','Second estimation','Corrected second estimation');
hold off;

%%

save('AnchorCalibration.mat','c','GroundTruthAnchorRanges','FirstRanges','SecondRanges','ThirdRanges','Offset');

%%

rosinit;
pause(1);
RostopicAnchor = '/vicon/Anchors_Johann/Anchors_Johann';
ViconAnchorsSubscriber = rossubscriber(RostopicAnchor);

%%

input("Change Vicon");
[a,b]=getAnchorsGroundTruth(ViconAnchorsSubscriber);
c = getAnchorsPosition(6,a,b);
scatter3(c(1,:),c(2,:),c(3,:),5,"bo");
hold on;
rosshutdown;

%%

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

PortAddress = seriallist;
SerialObject = serial(PortAddress);

AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,100,6);
AnchorPositions = AnchorCalibration(AnchorRangeMean,6);
scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),7,"ro");

%%

AnchorRangeMean = AnchorRangeMean + Offset*1000;
AnchorPositions = AnchorCalibration(AnchorRangeMean,6);
scatter3(AnchorPositions(:,1),AnchorPositions(:,2),AnchorPositions(:,3),8,"ko");

legend('GT','Estimation','Corrected estimation');
hold off;