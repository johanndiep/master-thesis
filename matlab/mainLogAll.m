% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script logs all the necessary data for offline analysis.
%   - VICON data:
%       - Anchor system position and attitude in VICON frame
%       - Anchor marker tags in object body frame
%       - Bebop position and attitude in VICON frame
%       - Bebop marker tag in object body frame
%   - Ultra-Wideband data:
%       - Range measurements between anchors during anchor calibration
%       - Calculated anchor positions
%       - Range measurements between tag and each anchor

%% Execution

% Setup
SerialObject = SerialPortSetup([]);
[ViconAnchorSubscriber,ViconDroneSubscriber] = ROSCommunication();
[NumberOfIterations,~,NumberOfIterationsForCalibration,NumberOfAnchors,AnchorMarkers,TagMarker] = DesiredParameters();

% Anchor ranging, calibration and ground-truth
input("Placing anchors and changing connected module in Sniffer mode.");
AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors);
AnchorPositions = AnchorCalibration(AnchorRangeMean,NumberOfAnchors);
[AnchorPositionGroundTruth,AnchorQuaternionGroundTruth] = getAnchorGroundTruth(ViconAnchorSubscriber);
AnchorGroundTruthPositions = getAnchorPosition(NumberOfAnchors,AnchorPositionGroundTruth,AnchorQuaternionGroundTruth,AnchorMarkers);

% Tag ranging
input("Placing Bebop drone and prepare for flying.");
SerialObject = SerialPortSetup(SerialObject);
[TimeArray,RangeArray,DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray] = logRangeMeasurement(SerialObject,ViconDroneSubscriber,NumberOfIterations,NumberOfAnchors);

% Save and closing 
save('rangemeasurement.mat','AnchorPositions','TimeArray','RangeArray','DronePositionGroundTruthArray','DroneQuaternionGroundTruthArray','TagMarker'); % saving to workspace
rosshutdown();