% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function reads the position ground-truth data of the object from the 
% VICON system over a ROS bridge. In order to execute this function, the 
% corresponding ROS driver for the VICON motion capture system needs to be 
% launched (more informations on https://github.com/ethz-asl/vicon_bridge).
%
% Input:
%   - ViconSubscriber: Subscriber object to a ROS topic
% 
% Output:
%   - ViconPos: Position of the object body-fixed frame in VICON inertial 
%     coordinate frame in form (3 x 1)
%   - ViconQuat: Quaternion of the object body-fixed frame relative to the 
%     VICON inertial coordinate frame in form (4 x 1)

function [ViconPos,ViconQuat] = getGroundTruth(ViconSubscriber)
        Message = ViconSubscriber.LatestMessage;
        
        x = Message.Transform.Translation.X;
        y = Message.Transform.Translation.Y;
        z = Message.Transform.Translation.Z;
        
        qw = Message.Transform.Rotation.W;
        qx = Message.Transform.Rotation.X;
        qy = Message.Transform.Rotation.Y;
        qz = Message.Transform.Rotation.Z;
        
        ViconPos = [x;y;z];
        ViconQuat = [qw;qx;qy;qz];
end