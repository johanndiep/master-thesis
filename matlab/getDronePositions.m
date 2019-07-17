% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function calculates the position of the drone in the world-frame.
%
% Input:
%   - DronePositionGroundTruthArray: Stores the ground-truth positions 
%   - DroneQuaternionGroundTruthArray: Stores the ground-truth quaternions
%   - TagMarker: Coordinates of the tag marker in Bebop body-frame
%
% Output:
%   - DroneGroundTruthPositions: The position of the drone in the world-frame

function DroneGroundTruthPositions = getDronePositions(DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,TagMarker)
    X_ViconToWorld = 0.229;
    Y_ViconToWorld = 0.246;
    Z_ViconToWorld = -0.243;
    T_ViconToWorld = [1,0,0,X_ViconToWorld;0,1,0,Y_ViconToWorld;0,0,1,Z_ViconToWorld;0,0,0,1];

    for i = 1:size(DronePositionGroundTruthArray,2)
        A = quat2rotm(DroneQuaternionGroundTruthArray(:,i)');
        b = DronePositionGroundTruthArray(:,i);
        DroneGroundTruthPositions(:,i) = T_ViconToWorld * [A,b;0,0,0,1] * [TagMarker';1];
    end
    DroneGroundTruthPositions(4,:) = [];
end