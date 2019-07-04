% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This script is for checking the angular readout via VICON.

rosinit;
RosTopicDrone = '/vicon/Bebop_Johann/Bebop_Johann';
RosTopicAnchors = '/vicon/Anchors_Johann/Anchors_Johann';
ViconDroneSubscriber = rossubscriber(RosTopicDrone); % creating subscriber object for the drone
ViconAnchorsSubscriber = rossubscriber(RosTopicAnchors);  % subscriber object for anchors network
pause(2); % time needed for initialization

TagMarker = [-20.7756;34.7541;87.6537]/1000;
AnchorMarker = [22.6417;3.00382;87.7027]/1000;

while true 
   [Dp,Dq] = getGroundTruth(ViconDroneSubscriber);
   TagViconFrame = getCoordinateViconFrame(Dq,Dp,TagMarker);
   RotationAngles = atan2(2*(Dq(1)*Dq(4)+Dq(2)*Dq(3)),(1-2*(Dq(3)^2+Dq(4)^2)));
   RotationAngles = RotationAngles/(2*pi)*360;
   [Ap,Aq] = getAnchorsGroundTruth(ViconAnchorsSubscriber);
   AnchorViconFrame = getCoordinateViconFrame(Aq,Ap,AnchorMarker);
   AnchorRotationAngles = atan2(2*(Aq(1)*Aq(4)+Aq(2)*Aq(3)),(1-2*(Aq(3)^2+Aq(4)^2)));
   AnchorRotationAngles = AnchorRotationAngles/(2*pi)*360;
end

rosshutdown;