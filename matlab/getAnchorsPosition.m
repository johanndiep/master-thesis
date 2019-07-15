% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function calculates the position of the anchors in the world-frame.
%
% Input:
%   - NumberOfAnchors: Amount of anchors in the setup 
%   - AnchorsPositionGroundTruth: Position of the anchors body-fixed frame in VICON inertial coordinate frame
%   - AnchorsQuaternionGroundTruth: Quaternion of the anchors body-fixed frame relative to the VICON inertial coordinate frame
%
% Output:
%   - AnchorsGroundTruth: The position of the anchors in the world-frame

function AnchorsGroundTruth = getAnchorsPosition(NumberOfAnchors,AnchorsPositionGroundTruth,AnchorsQuaternionGroundTruth)
    X_ViconToWorld = 0.229;
    Y_ViconToWorld = 0.246;
    Z_ViconToWorld = -0.243;
    T_ViconToWorld = [1,0,0,X_ViconToWorld;0,1,0,Y_ViconToWorld;0,0,1,Z_ViconToWorld;0,0,0,1];
    
    HeightTop = 2.44-0.284; % top anchors heights measured from bottom anchor

    Deviation = [-0.045,HeightTop-0.045,-0.04,HeightTop-0.04,-0.04,HeightTop-0.04];
    
    MarkerBodyFrame = [-1081.13,-1243.85,24.7311;-1081.13,-1243.85,24.7311;2203.3,-1283.79,-62.378;2203.3,-1283.79,-62.378;-1122.16,2527.63,37.647;-1122.16,2527.63,37.647]/1000;
    AnchorRotationGroundTruth = quat2rotm(AnchorsQuaternionGroundTruth');
    
    for i = 1:NumberOfAnchors
        AnchorsGroundTruth(:,i) = T_ViconToWorld * [AnchorRotationGroundTruth,AnchorsPositionGroundTruth;0,0,0,1] * [MarkerBodyFrame(i,:)';1] + [0;0;Deviation(i);0];
    end
    AnchorsGroundTruth(4,:) = [];
end