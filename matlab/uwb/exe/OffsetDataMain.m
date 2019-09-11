% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% With this script, UWB range offset measurement can be gathered while the drone
% is commanded to fly a circular trajectory. The control part is copied from
% "CircleControl.m".
%
% Placement of the anchors with the following assumptions:
%   - Anchor 1 is set to be the origin of the coordinate system
%   - Anchor 3 and 5 are fixed on the same height as anchor 1
%   - Anchor 2, 4 and 6 are fixed at a known constant height
%   - Anchor 5 is assumed to be on the same axis with anchor 1 
%     without loss of generality
%   - Top anchors are assumed to have same x/y-coordinates as 
%     bottom anchors
% 
% The anchors are distributed as follows where p1/p2/p3 are the unknown parameters:
%   - Pole 1: Anchor 1 (0,0,0), Anchor 2 (0,0,h)
%   - Pole 2: Anchor 3 (p1,p2,0), Anchor 4 (p1,p2,h)
%   - Pole 3: Anchor 5 (0,p3,0), Anchor 6 (0,p3,h)
%
% In order to optimize the performance, the following parameters 
% need to be tuned:
%   - P/D-gains in "Controller.m"
%   - Threshold for maximal rotation in "Controller.m"
%   - Time interval between each EKF iteration
%   - x/P-initialization in "ConstantVelocityEKF.m"
%   - R/Q-covariance in "ConstantVelocityEKF.m"
%   - Goal state changing rate f in "TrajectoryGenerator.m"
%   - Absolute goal velocity in "TrajectoryGenerator.m"
%
% Furthermore, the following points need to be investigated:
%   - The yaw correction method could be optimized.
%   - Are the buttons of the Spacemouse fast enough to react?
%     [VICON readings and iterations occur at high frequency. However, the UWB
%     readings occur at lower frequency. Therefore, it can happen, that 
%     MATLAB does not respond to the button click. By holding the button, 
%     the script has time to execute the landing maneuver.]
%   - Tune the time-variant goal velocities and goal state rate 
%     such that the flight is smooth. Is goal velocities and goal state rate 
%     coupled?
%     [I believe that a mismatch between goal state rate, goal and current
%      velocity is responsible for shaky flights.]
%   - Any delays introduced due to reading processing?
%   - Does the delay in range reading influence position estimation?
%   - Does a batch of range measurement match to the current position good enough?
%   - Should zero measurements be rejected?
%   - Closing procedure right?
%   - Set tag number, check if this number is interrogated.
%
% Step-by-Step:
%   1. Place pole 1 and pole 3 such that the corresponding anchors have 
%      the same orientation.
%   2. Place pole 2 such that its anchors are opposite directed compared to 
%      the anchors from pole 1/3 and have positive x/y-coordinates.
%   3. Power up each anchor, the modules should be in Anchor mode already.
%      This can also be tested in Terminal using the following picocom 
%      command and a computer-attached Sniffer module: sudo picocom /dev/ttyACM*
%   4. Connect the Sniffer node to the computer and run the following
%      command in terminal: sudo chmod 666 /dev/ttyACM*
%   6. Calibrate the VICON system and place the origin in the middle of the room 
%      with the T-link.
%   8. Place the drone such that the body-fixed frame (x-forward,y-left,z-ascend)
%       is aligned with the VICON frame.
%   9. Attach VICON markers on the Bebop and on the UWB antenna ("TagMarker"), 
%      group the markers on the VICON Tracker to an object and name it 
%      "Bebop_Johann".
%   10. Attach VICON markers on the anchor poles ("Pole1", "Pole2" and "Pole3"),
%      group the markers on the VICON Tracker to an object and name it 
%      "Anchors_Johann".
%   11. Write down the body-frame coordinates of "TagMarker", "Pole1", "Pole2" 
%       and "Pole3". The coordinates can be seen in VICON Tracker application.
%   12. Connect the computer with the VICON machine via Ethernet
%   13. Turn on the Bebop and connect the laptop with it over Wi-Fi.
%   14. Turn on the Spacemouse and start its ROS driver.
%   15. Start the ROS VICON bridge node.
%   16. Start the ROS driver for the Bebop.
%   17. Set the desired circle parameters.
%   18. Run the following script.
%
% To-Do:
%   - Track antenna instead of object center of mass with VICON.
%   - Figure out how to subscribe to "/vicon/markers".

clear; clc;

rosshutdown; rosinit;

%% Parameters

% marker positions
Marker.Dev = [-0.04,2.116];
Marker.MarkP1 = [-1313,-1684,38]/1000;
Marker.MarkP2 = [2728,-134,-64]/1000;
Marker.MarkP3 = [-1414,1819,26]/1000;
Marker.MarkTag = [20,6,37]/1000;

% initialize the trajectory object
MidPoint = [0,0];
Height = 1;
AbsVel = 0;
Radius = 1;
Frequency = 0.01; % one circle in 100 seconds
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

%% Preliminary

% initializing a range measurement object
RangeMeasObj = RangeMeasurements();

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');
VicAncSub = rossubscriber('/vicon/Anchors_Johann/Anchors_Johann');

% initializing a controller object
ControlObj = Controller();

% pre-allocation
SaveViconPos = zeros(3,5000);
SaveViconQuat = zeros(4,5000);
SaveCurPos = zeros(3,5000);
SaveCurVel = zeros(3,5000);
SaveGoalPos = zeros(3,5000);
SaveRangeArr = zeros(6,5000);

% anchor positions
[VicAncPos,VicAncQuat] = getGroundTruth(VicAncSub);

%% PID

% starting the drone after the left button on the Spacemouse is pushed
JoyMessage = JoySub.LatestMessage;
while JoyMessage.Buttons(1) == 0
   JoyMessage = JoySub.LatestMessage; 
end

ControlObj.Start; % starting the drone
pause(5);

% initializing the constant velocity modeled EKF
Model = ConstantVelocityEKF();
tic;

i = 1;
while true
    % stopping estimation when the right button on the Spacemouse is pushed
    JoyMessage = JoySub.LatestMessage;
    if JoyMessage.Buttons(2) == 1
        break;
    end
    
    % Vicon ground-truth position and quaternion of the drone
    [ViconPos,ViconQuat] = getGroundTruth(VicDroneSub);
    
    % Reading UWB range measurements
    RangeArray = RangeMeasObj.TagAnchorRanging;
    
    % prior and posterior update with process and measurement model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
    tic;
    
    % time-variant goal position and velocity
    [GoalPos,GoalVel] = TrajObj.getCircleTrajectory(Time);
    
    % moving the drone towards the desired goal position while 
    % keeping the orientation fixed
    ControlObj.NoTurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat);
    
    % saving to array
    SaveViconPos(:,i) = ViconPos;
    SaveViconQuat(:,i) = ViconQuat;
    SaveCurPos(:,i) = CurPos;
    SaveCurVel(:,i) = CurVel;
    SaveGoalPos(:,i) = GoalPos';
    SaveRangeArr(:,i) = RangeArray;
    i = i+1;
end

ControlObj.End; % landing the drone
pause(5);

rosshutdown;

%%

CuttingIndex = find(SaveViconPos(1,:),1,'last')+1;
SaveViconPos(:,CuttingIndex:end) = [];
SaveViconQuat(:,CuttingIndex:end) = [];
SaveCurPos(:,CuttingIndex:end) = [];
SaveCurVel(:,CuttingIndex:end) = [];
SaveGoalPos(:,CuttingIndex:end) = [];
SaveRangeArr(:,CuttingIndex:end) = [];

save('UWB-GP.mat','SaveViconPos','SaveViconQuat', ...
    'SaveCurPos','SaveCurVel','SaveGoalPos','SaveRangeArr', ...
    'VicAncPos','VicAncQuat','Marker');

clear; clc;