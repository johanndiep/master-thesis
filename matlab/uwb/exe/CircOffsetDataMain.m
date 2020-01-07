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
%   - x/P-initialization in "ConstantVelocityEKF.m"
%   - R/Q-covariance in "ConstantVelocityEKF.m"
%   - Goal state changing rate f in "TrajectoryGenerator.m"
%
% Furthermore, the following points need to be investigated:
%   - The yaw correction method could be optimized.
%     [By grouping translation and rotation in one command, jiggly
%     movementes can be avoided.]
%   - Are the buttons of the Spacemouse fast enough to react?
%     [VICON readings and iterations occur at high frequency. However, the UWB
%     readings occur at lower frequency. Therefore, it can happen, that 
%     MATLAB does not respond to the button click. By holding the button, 
%     the script has time to execute the landing maneuver. Sometimes, it also
%     happens that the landing button does not work. In that case, send a separate
%     command from the terminal.]
%   - Tune the time-variant goal velocities and goal state rate 
%     such that the flight is smooth. Is goal velocities and goal state rate 
%     coupled?
%     [An equation is relating frequency to absolute velocities now.]
%   - Is the dT timing accurate?
%   - Any delays introduced due to reading processing?
%   - Does the delay in range reading influence position estimation?
%   - Does a batch of range measurement match to the current position good enough?
%   - Should zero measurements be rejected?
%   - Closing procedure right?
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
%   10. Connect the computer with the VICON machine via Ethernet
%   11. Turn on the Bebop and connect the laptop with it over Wi-Fi.
%   12. Turn on the Spacemouse and start its ROS driver.
%   13. Start the ROS VICON bridge node.
%   14. Start the ROS driver for the Bebop.
%   15. Set the desired circle parameters.
%   16. Run the following script.
%
% To-Do:
%   - Figure out how to subscribe to "/vicon/markers".
%   - Set tag number, check if this number is interrogated.

clear; clc;

%rosshutdown; 
rosinit;

load('AnchorPos.mat'); % load the anchor positions if available

%% Parameters

% coordinate transformation
T = diag(ones(1,4));
T(1:3,4) = [-0.23;-0.25;0.25];
A = T*[AnchorPos';ones(1,6)]; AnchorPos = A(1:3,:)';

% initialize the trajectory object
MidPoint = [2,1.5];
Height = 1;
Radius = 1.5;
Frequency = 1/90;
AbsVel = 2*Radius*pi*Frequency;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency
ChangeHeading = true; % drone is pointing in the direction of flight
PointToCenter = false; % able to face to the center of the circle
SplineFlight = false; % do not change this variable

%% Preliminary

% initializing a range measurement object
RangeMeasObj = RangeMeasurements();

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller(FastModus);

% pre-allocation
SaveViconPos = zeros(3,1000);
SaveViconQuat = zeros(4,1000);
SaveCurPos = zeros(3,1000);
SaveCurVel = zeros(3,1000);
SaveGoalPos = zeros(3,1000);
SaveGoalVel = zeros(3,1000);
SaveRangeArr = zeros(6,1000);

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
    
    % prior update with process model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    
    % posterior update with measurement model
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
    tic;
    
    if ChangeHeading == false
        % time-variant goal position and velocity
        [GoalPos,GoalVel] = TrajObj.getCircleTrajectory(Time);
        
        % moving the drone towards the desired goal position while
        % keeping the orientation fixed
        ControlObj.NoTurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat);
    else
        if PointToCenter == false
            % time-variant goal position, yaw and velocity
            [GoalPos,GoalYaw,GoalVel] = TrajObj.getYawCircleTraj(Time);
        else
            % time-variant goal position, yaw and velocity
            [GoalPos,GoalYaw,GoalVel] = TrajObj.getCenCircleTraj(Time);
        end
        
        % moving the drone towards the desired goal position while 
        % keeping the orientation in the direction of flight
        ControlObj.TurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat,GoalYaw);        
    end
    
    % saving to array
    SaveViconPos(:,i) = ViconPos;
    SaveViconQuat(:,i) = ViconQuat;
    SaveCurPos(:,i) = CurPos;
    SaveCurVel(:,i) = CurVel;
    SaveGoalPos(:,i) = GoalPos';
    SaveGoalVel(:,i) = GoalVel';
    SaveRangeArr(:,i) = RangeArray;
    i = i+1;
end

ControlObj.End; % landing the drone
pause(5);

rosshutdown;

CuttingIndex = find(SaveViconPos(1,:),1,'last')+1;
SaveViconPos(:,CuttingIndex:end) = [];
SaveViconQuat(:,CuttingIndex:end) = [];
SaveCurPos(:,CuttingIndex:end) = [];
SaveCurVel(:,CuttingIndex:end) = [];
SaveGoalPos(:,CuttingIndex:end) = [];
SaveGoalVel(:,CuttingIndex:end) = [];
SaveRangeArr(:,CuttingIndex:end) = [];

if ChangeHeading == false
    save('UWBCircConDataGP.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
        'AnchorPos','MidPoint','Height','Radius','ChangeHeading','PointToCenter', ...
        'SplineFlight');
else
    if PointToCenter == false
        save('UWBYawCircConDataGP.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
            'AnchorPos','MidPoint','Height','Radius','ChangeHeading','PointToCenter', ...
            'SplineFlight');
    else
        save('UWBCenCircConDataGP.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
            'AnchorPos','MidPoint','Height','Radius','ChangeHeading','PointToCenter', ...
            'SplineFlight');
    end
end

clear; clc;