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

    Deviation = [0,HeightTop,0,HeightTop,0,HeightTop];
    
    MarkerBodyFrame = [-976.549,-1054.63,17.3756;-976.549,-1054.63,17.3756;1934.5,-1107.24,-52.7711;1934.5,-1107.24,-52.7711;-957.95,2161.86,35.3954;-957.95,2161.86,35.3954]/1000;
    AnchorRotationGroundTruth = quat2rotm(AnchorsQuaternionGroundTruth');
    
    for i = 1:NumberOfAnchors
        AnchorsGroundTruth(:,i) = T_ViconToWorld * [AnchorRotationGroundTruth,AnchorsPositionGroundTruth;0,0,0,1] * [MarkerBodyFrame(i,:)';1] + [0;0;Deviation(i);0];
    end
    AnchorsGroundTruth(4,:) = [];
end