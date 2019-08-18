% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function reads the position ground-truth data of the drone from the 
% VICON system over a ROS bridge. In order to execute this function, the 
% corresponding ROS driver for the VICON motion capture system needs to be 
% launched (more informations on https://github.com/ethz-asl/vicon_bridge).
%
% Input:
%   - ViconDroneSubscriber: Subscriber object to '/vicon/Bebop_Johann/Bebop_Johann' topic
% 
% Output:
%   - DronePositionGroundTruth: Position of the drone body-fixed frame in VICON inertial coordinate frame
%   - DroneQuaternionGroundTruth: Quaternion of the drone body-fixed frame relative to the VICON inertial coordinate frame

function [DronePositionGroundTruth,DroneQuaternionGroundTruth] = getGroundTruth(ViconDroneSubscriber)
        LatestMessage = ViconDroneSubscriber.LatestMessage;    
        DronePositionGroundTruth = [LatestMessage.Transform.Translation.X;LatestMessage.Transform.Translation.Y;LatestMessage.Transform.Translation.Z];
        DroneQuaternionGroundTruth = [LatestMessage.Transform.Rotation.W;LatestMessage.Transform.Rotation.X;LatestMessage.Transform.Rotation.Y;LatestMessage.Transform.Rotation.Z];
end