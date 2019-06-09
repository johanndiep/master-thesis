% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This function reads the position ground-truth data from the VICON system
% over a ROS bridge.

function [drone_position_gt,drone_quaternion_gt] = getGroundTruth(ViconSub_pos)
    %% Reading position data from ROS stream

        msg = ViconSub_pos.LatestMessage;
        
        drone_position_gt = [msg.Transform.Translation.X; ...
            msg.Transform.Translation.Y; ...
            msg.Transform.Translation.Z]; % position
        
        drone_quaternion_gt = [msg.Transform.Rotation.X; ...
            msg.Transform.Rotation.Y; ...
            msg.Transform.Rotation.Z; ...
            msg.Transform.Rotation.W]; % orientation
end