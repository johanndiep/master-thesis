% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This function reads the position ground-truth data from the VICON system
% over a ROS bridge.

function drone_position_gt = getGroundTruth()
    %% ROS communication initialization, plotting setup
        
    figure();
    hold on;
    title("VICON coordinate system");
    xlabel("x-Axis [m]");
    ylabel("y-Axis [m]");
    zlabel("z-Axis [m]");
    grid on
    
    %% Reading position data and plotting
    
    rostopic = '/vicon/Johann_Bebop/Johann_Bebop'; % listening to topic
    ViconPos_sub = rossubscriber(rostopic); % creating subscriber object
    pause(5);
    
    while true
        % reading current position and plotting
        msg = ViconPos_sub.LatestMessage;
        drone_position = [msg.Transform.Translation.X; ...
            msg.Transform.Translation.Y; ...
            msg.Transform.Translation.Z];
        
        scatter3(drone_position(1),drone_position(2),drone_position(3),5,'r'); % plotting waypoints
        drawnow
    end
end