% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% With this script, the drone is commanded to fly a circular trajectory, where
% the standard UWB range measurement model is used. The control part is copied 
% from "CircleControl.m".
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
%   - How often does zero range measurements occur? What happens if a large 
%     part of the batch consists of zero measurements? 
%     [In this case, zero measurements are removed from Z, h, H and R.]
%   - How about other kind of outliers?
%   - Right now, measurement starts at anchor 1 and ends at anchor
%     6. Maybe one can speed up the frequency by considering the
%     next 6 available range measurement instead.
%   - Any delays introduced due to reading processing?
%   - Does the delay in range reading influence position estimation?
%   - Does a batch of range measurement match to the current position good enough?
%   - Closing procedure right?
%   - How to accomodate for ranging offset from the UWB modules?
%     [Learning the offset via Gaussian Process.]
%   - For more points on anchor calibration, see "AnchorCalibMain.m".
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
%   5. Run "AnchorCalibMain.m" first in order to obtain all the anchor
%      positions in a file called "AnchorPos.mat".
%   6. Calibrate the VICON system and place the origin in the room with the
%      T-link, here the T-link should be placed at the corner of pole 1.
%      Thereby, there should not be any rotations between anchor and VICON
%      frame.
%   7. Measure the coordinates of anchor 1 in the VICON frame and fill the
%      transformation matrix T.
%   8. Attach VICON markers on the Bebop, group the markers on the VICON
%      Tracker to an object and name it "Bebop_Johann"
%   9. Place the drone such that the body-fixed frame (x-forward,y-left,z-ascend)
%      is aligned with the VICON frame
%   10. Connect the computer with the VICON machine via Ethernet
%   11. Turn on the Bebop and connect the laptop with it over Wi-Fi
%   12. Turn on the Spacemouse and start its ROS driver.
%   13. Start the ROS VICON bridge node
%   14. Start the ROS driver for the Bebop
%   15. Set the desired circle parameters
%   16. Run the following script
%
% To-Do:
%   - Track marker instead of drone center with VICON. [x]
%   - Set tag number, check if this number is interrogated.

clear; clc;

rosinit;

load('AnchorPos.mat'); % load the anchor positions if available

%% Parameters

MarkTag = [-6.15657,6.14709,78.9883]/1000; % body-frame coordinate of tag antenna

% coordinate transformation
T = diag(ones(1,4));
T(1:3,4) = [-0.23;-0.25;0.25];
A = T*[AnchorPos';ones(1,6)]; AnchorPos = A(1:3,:)';

% initialize the trajectory object
MidPoint = [2,1.5];
Height = 1;
Radius = 1.25;
Frequency = 1/25;
AbsVel = 2*Radius*pi*Frequency;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency
ChangeHeading = true; % able to change its direction during flight
PointToCenter = true; % able to face to the center of the circle

%% Preliminary

% initializing a range measurement object
RangeMeasObj = RangeMeasurements();

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller(FastModus);

% pre-allocation
SaveViconPos = zeros(3,600);
SaveViconQuat = zeros(4,600);
SaveCurPos = zeros(3,600);
SaveCurVel = zeros(3,600);
SaveGoalPos = zeros(3,600);
SaveGoalVel = zeros(3,600);
SaveRangeArr = zeros(6,600);
SaveTime = zeros(1,600);

%% PID

% starting the drone after the left button on the Spacemouse is pushed
JoyMessage = JoySub.LatestMessage;
while JoyMessage.Buttons(1) == 0
   JoyMessage = JoySub.LatestMessage; 
end

ControlObj.Start; % starting the drone
pause(5);

% initializing the constant velocity modeled EKF
RangeArray = RangeMeasObj.TagAnchorRanging/1000;
Model = ConstantVelocityUWB(AnchorPos,RangeArray);
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
    RangeArray = RangeMeasObj.TagAnchorRanging/1000;    
    
    % prior update with process model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    
    % posterior update with measurement model
    [CurPos,CurVel] = Model.UpdateMeasurement(RangeArray);
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
    SaveTime(i) = Time;
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
SaveTime(CuttingIndex:end) = [];

if ChangeHeading == false
    save('UWBCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
        'AnchorPos','MarkTag','MidPoint','ChangeHeading','PointToCenter',...
        'Radius','SaveTime');
else
    if PointToCenter == false
    save('UWBYawCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
        'AnchorPos','MarkTag','MidPoint','ChangeHeading','PointToCenter',...
        'Radius','SaveTime');
    else
    save('UWBCenCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
        'AnchorPos','MarkTag','MidPoint','ChangeHeading','PointToCenter',...
        'Radius','SaveTime');        
    end
end

clear; clc;

%% Subplot 1: Trajectory

% load('UWBYawCircConData.mat');

figure();

subplot(1,2,1);
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-Radius-1,MidPoint(1)+Radius+1]);
ylim([MidPoint(2)-Radius-1,MidPoint(2)+Radius+1]);
zlim([0,2.5]);
hold on;

SaveViconPos = SaveViconPos(:,1:2:end);
SaveViconQuat = SaveViconQuat(:,1:2:end);

scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:)*0,1,'k.');

Angle = 0:0.01:2*pi;
x = MidPoint(1)+Radius*cos(Angle);
y = MidPoint(2)+Radius*sin(Angle) ;
z = zeros(size(x));
plot3(x,y,z,'r-')

legend('Projected Ground-Truth','Reference');
set(0,'DefaultLegendAutoUpdate','off')

RotMats = quat2rotm(SaveViconQuat');
Xb = permute(RotMats(:,1,:),[1,3,2]);
Yb = permute(RotMats(:,2,:),[1,3,2]);
Zb = permute(RotMats(:,3,:),[1,3,2]);
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Xb(1,:),Xb(2,:),Xb(3,:),0.3,'r');
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Yb(1,:),Yb(2,:),Yb(3,:),0.3,'g');
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Zb(1,:),Zb(2,:),Zb(3,:),0.3,'b');

grid on;
view(35.1654,48.1915)

clear; clc;

%% Subplot 2: Trajectory

% load('UWBCenCircConData.mat');

subplot(1,2,2);
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-Radius-1,MidPoint(1)+Radius+1]);
ylim([MidPoint(2)-Radius-1,MidPoint(2)+Radius+1]);
zlim([0,2.5]);
hold on;

SaveViconPos = SaveViconPos(:,1:2:end);
SaveViconQuat = SaveViconQuat(:,1:2:end);

scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:)*0,1,'k.');

Angle = 0:0.01:2*pi;
x = MidPoint(1)+Radius*cos(Angle);
y = MidPoint(2)+Radius*sin(Angle) ;
z = zeros(size(x));
plot3(x,y,z,'r-')

legend('Projected Ground-Truth','Reference');
set(0,'DefaultLegendAutoUpdate','off')

RotMats = quat2rotm(SaveViconQuat');
Xb = permute(RotMats(:,1,:),[1,3,2]);
Yb = permute(RotMats(:,2,:),[1,3,2]);
Zb = permute(RotMats(:,3,:),[1,3,2]);
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Xb(1,:),Xb(2,:),Xb(3,:),0.3,'r');
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Yb(1,:),Yb(2,:),Yb(3,:),0.3,'g');
quiver3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:), ...
    Zb(1,:),Zb(2,:),Zb(3,:),0.3,'b');

grid on;
view(35.1654,48.1915)

hold off;

%% Position Graph: Trajectory 1

% load('UWBYawCircConData.mat');

figure();

subplot(3,2,1);
title("Heading to Flying Direction",'FontWeight','Normal');
hold on;
plot(SaveTime,SaveGoalPos(1,:)-SaveViconPos(1,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(1,:)-SaveCurPos(1,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("x-Error [m]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.25,0.5]);
hold off;

subplot(3,2,3);
hold on;
plot(SaveTime,SaveGoalPos(2,:)-SaveViconPos(2,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(2,:)-SaveCurPos(2,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("y-Error [m]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.25,0.3]);
hold off;

subplot(3,2,5);
hold on;
plot(SaveTime,SaveGoalPos(3,:)-SaveViconPos(3,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(3,:)-SaveCurPos(3,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("z-Error [m]");
xlabel("Time [s]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.3,0.25]);
hold off;

clear; clc;

%% Position Graph: Trajectory 2

% load('UWBCenCircConData.mat');

subplot(3,2,2);
title("Heading to Center",'FontWeight','Normal');
hold on;
plot(SaveTime,SaveGoalPos(1,:)-SaveViconPos(1,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(1,:)-SaveCurPos(1,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.5,0.5]);
hold off;

subplot(3,2,4);
hold on;
plot(SaveTime,SaveGoalPos(2,:)-SaveViconPos(2,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(2,:)-SaveCurPos(2,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.5,0.5]);
hold off;

subplot(3,2,6);
hold on;
plot(SaveTime,SaveGoalPos(3,:)-SaveViconPos(3,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(3,:)-SaveCurPos(3,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlabel("Time [s]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.3,0.3]);
hold off;