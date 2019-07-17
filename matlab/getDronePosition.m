% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function returns the position of the selected marker in the Vicon
% frame. 
%
% Input:
%   - DroneQuaternionGroundTruth: Quaternion of the subscribed object in the form (4,1)
%   - DronePositionGroundTruth: Position of the subsribed object in the form (3,1)
%   - TagMarker: Position of the marker in the bodyframe of the object in the form (3,1)
%
% Output:
%   - DroneGroundTruthPosition: World frame position of the marker in the form (3,1)

function DroneGroundTruthPosition = getDronePosition(DronePositionGroundTruth,DroneQuaternionGroundTruth,TagMarker)
    X_ViconToWorld = 0.229;
    Y_ViconToWorld = 0.246;
    Z_ViconToWorld = -0.243;
    T_ViconToWorld = [1,0,0,X_ViconToWorld;0,1,0,Y_ViconToWorld;0,0,1,Z_ViconToWorld;0,0,0,1];    

    DroneRotationGroundTruth = quat2rotm(DroneQuaternionGroundTruth');
    DroneGroundTruthPosition = T_ViconToWorld * [DroneRotationGroundTruth,DronePositionGroundTruth;0,0,0,1] * [TagMarker';1];
    DroneGroundTruthPosition(4) = [];
end