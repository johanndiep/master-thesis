% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function reads the position ground-truth data of the anchor network
% from the VICON system over a ROS bridge. In order to execute this 
% function, the corresponding ROS driver for the VICON motion capture 
% system needs to be launched (more informations on 
% https://github.com/ethz-asl/vicon_bridge).
%
% Input:
%   - ViconAnchorSubscriber: Subscriber object to '/vicon/Anchors_Johann/Anchors_Johann' topic
% 
% Output:
%   - AnchorPositionGroundTruth: Position of the anchors body-fixed frame in VICON inertial coordinate frame
%   - AnchorQuaternionGroundTruth: Quaternion of the anchors body-fixed frame relative to the VICON inertial coordinate frame

function [AnchorPositionGroundTruth,AnchorQuaternionGroundTruth] = getAnchorGroundTruth(ViconAnchorSubscriber)
        LatestMessage = ViconAnchorSubscriber.LatestMessage;
        AnchorPositionGroundTruth = [LatestMessage.Transform.Translation.X;LatestMessage.Transform.Translation.Y;LatestMessage.Transform.Translation.Z];
        AnchorQuaternionGroundTruth = [LatestMessage.Transform.Rotation.W;LatestMessage.Transform.Rotation.X;LatestMessage.Transform.Rotation.Y;LatestMessage.Transform.Rotation.Z];
end