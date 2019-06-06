% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This function reads the position ground-truth data from the VICON system
% over a ROS bridge.

function tag_position_gt = getGroundTruth(ViconSub_pos)
    %% Reading position data from ROS stream

        % reading current position and plotting
        msg = ViconSub_pos.LatestMessage;
        tag_position_gt = [msg.Transform.Translation.X; ...
            msg.Transform.Translation.Y; ...
            msg.Transform.Translation.Z];
end