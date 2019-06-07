% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This function reads the ground-truth position of the anchors network from the VICON system
% over a ROS bridge.

function [anchors_position_gt,anchors_quaternion_gt] = getAnchorsGroundTruth(ViconAnchorsSub_pos)
    %% Reading position data from ROS stream

        % reading current position
        msg = ViconAnchorsSub_pos.LatestMessage;
        anchors_position_gt = [msg.Transform.Translation.X; ...
            msg.Transform.Translation.Y; ...
            msg.Transform.Translation.Z];
        anchors_quaternion_gt = [msg.Transform.Rotation.X; ...
            msg.Transform.Rotation.Y; ...
            msg.Transform.Rotation.Z; ...
            msg.Transform.Rotation.W];
end