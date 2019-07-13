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
    X_ViconToWorld = 0.23;
    Y_ViconToWorld = 0.245;
    Z_ViconToWorld = -0.24;
    
    Deviation = [-5/100,2.44-0.28-5/100,-24/100,2.44-0.28-24/100,-13.5/100,2.44-0.28-13.5/100];
    
    MarkerBodyFrame = [-1388.8,-1466.67,-67.1381;-1388.8,-1466.67,-67.1381;2697.54,-135.384,19.5984;2697.54,-135.384,19.5984;-1308.74,1602.05,47.5397;-1308.74,1602.05,47.5397]/1000;
    AnchorRotationGroundTruth = quat2rotm(AnchorsQuaternionGroundTruth');
    
    for i = 1:NumberOfAnchors
        AnchorsGroundTruth(:,i) = [AnchorRotationGroundTruth,AnchorsPositionGroundTruth;0,0,0,1] * [MarkerBodyFrame(i,:)';1] + [0;0;Deviation(i);0];
    end
    AnchorsGroundTruth(4,:) = [];
end