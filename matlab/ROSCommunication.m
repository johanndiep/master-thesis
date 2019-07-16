% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function sets up the ROS communication.
%
% Output:
%   - ViconAnchorSubscriber: Subscriber object for the anchor system
%   - ViconDroneSubscriber: Subscriber object for the Bebop drone

function [ViconAnchorSubscriber,ViconDroneSubscriber] = ROSCommunication()
    rosinit;
    pause(5);
    
    RostopicAnchor = '/vicon/Anchors_Johann/Anchors_Johann';
    RostopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
    
    ViconAnchorsSubscriber = rossubscriber(RostopicAnchor);
    ViconDroneSubscriber = rossubscriber(RostopicDrone); 
end