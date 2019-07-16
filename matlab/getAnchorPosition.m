% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function calculates the position of the anchors in the world-frame.
%
% Input:
%   - NumberOfAnchors: Amount of anchors in the setup 
%   - AnchorPositionGroundTruth: Position of the anchors body-fixed frame in VICON inertial coordinate frame
%   - AnchorQuaternionGroundTruth: Quaternion of the anchors body-fixed frame relative to the VICON inertial coordinate frame
%   - AnchorMarkers: Coordinates of the anchor markers in anchor system body-frame
%
% Output:
%   - AnchorGroundTruthPositions: The position of the anchors in the world-frame

function AnchorGroundTruthPositions = getAnchorPosition(NumberOfAnchors,AnchorPositionGroundTruth,AnchorQuaternionGroundTruth,AnchorMarkers)
    X_ViconToWorld = 0.229;
    Y_ViconToWorld = 0.246;
    Z_ViconToWorld = -0.243;
    T_ViconToWorld = [1,0,0,X_ViconToWorld;0,1,0,Y_ViconToWorld;0,0,1,Z_ViconToWorld;0,0,0,1];
    
    HeightTop = 2.44-0.284; % top anchors heights measured from bottom anchor
    Deviation = [-0.045,HeightTop-0.045,-0.04,HeightTop-0.04,-0.04,HeightTop-0.04];
    
    AnchorRotationGroundTruth = quat2rotm(AnchorQuaternionGroundTruth');
    
    for i = 1:NumberOfAnchors
        AnchorGroundTruthPositions(:,i) = T_ViconToWorld * [AnchorRotationGroundTruth,AnchorPositionGroundTruth;0,0,0,1] * [AnchorMarkers(i,:)';1] + [0;0;Deviation(i);0];
    end
    AnchorGroundTruthPositions(4,:) = [];
end