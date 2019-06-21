% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function returns the position of the selected marker in the Vicon
% frame. 
%
% Input:
%   - QuaternionGroundTruth: Quaternion of the subscribed object in the form (4,1)
%   - PositionGroundTruth: Position of the subsribed object in the form (3,1)
%   - MarkerBodyFrame: Position of the marker in the bodyframe of the object in the form (3,1)
%
% Output:
%   - PositionViconFrame: Vicon frame position of the marker in the form (3,1)

function PositionViconFrame = getCoordinateViconFrame(QuaternionGroundTruth,PositionGroundTruth,MarkerBodyFrame)
    RotationGroundTruth = quat2rotm(QuaternionGroundTruth');
    PositionViconFrame = [RotationGroundTruth,PositionGroundTruth;0,0,0,1] * [MarkerBodyFrame;1];
end