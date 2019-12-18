% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% With this script, the drone is commanded to fly a circular trajectory,
% where the GP-augmented UWB range measurement model is used. The control
% part is copied from "CircleControl.m".
%
% For setup, follow the instructions written in "OffsetDataMain.m". This
% program should be run after data gathering with "OffsetDataMain.m" and 
% hyperparameter training with "CovEvalMain.m".
% 
% In order to optimize the performance, the following parameters 
% need to be tuned:
%   - P/D-gains in "Controller.m"
%   - x/P-initialization in "ConstantVelocityEKF.m"
%   - R/Q-covariance in "ConstantVelocityEKF.m"
%   - Goal state changing rate f in "TrajectoryGenerator.m"
%
% To-Do:
%   - Figure out why there are mismatch at certain areas.
%     [The mismatch also happens at UWB flight.]

clear; clc;

rosinit;

% load('CircleHypGP.mat'); % load the parameters
load('YawCircleHypGP.mat'); % load the parameters with yaw
% load('CenCircleHypGP.mat'); % load the center parameters 

%% Parameters

% initialize the trajectory object
MidPoint = [2,1.5];
Height = 1;
Radius = 1.25;
Frequency = 1/25;
AbsVel = 2*Radius*pi*Frequency;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency
ChangeHeading = true; % drone pointing in direction of flight
PointToCenter = true; % able to face to the center of the circle

Kernel = @RBFKernel;
% Kernel = @DistanceKernel;
Mode = "GP";

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
Savet = zeros(6,600);
SaveAbs = zeros(6,600);
SaveCovVal = zeros(6,600);
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
Model = ConstantVelocityGP(X,Y,Kernel,NoiseStd,s0,s1,AnchorPos,RangeArray);
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
    [CurPos,CurVel,t,Abs,CovVal] = Model.UpdateMeasurement(RangeArray,Mode);
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
    Savet(:,i) = t;
    SaveAbs(:,i) = Abs;
    SaveCovVal(:,i) = CovVal;
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
SaveRangeArr(:,CuttingIndex:end) = [];
Savet(:,CuttingIndex:end) = [];
SaveAbs(:,CuttingIndex:end) = [];
SaveCovVal(:,CuttingIndex:end) = [];
SaveTime(CuttingIndex:end) = [];

if ChangeHeading == false
    save('GPCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel', ...
        'SaveRangeArr','Savet','SaveAbs','SaveCovVal','MidPoint', ...
        'ChangeHeading','PointToCenter','SaveTime','Radius');
else
    if PointToCenter == false
        save('GPYawCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel', ...
            'SaveRangeArr','Savet','SaveAbs','SaveCovVal','MidPoint', ...
            'ChangeHeading','PointToCenter','SaveTime','Radius');
    else
        save('GPCenCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel', ...
            'SaveRangeArr','Savet','SaveAbs','SaveCovVal','MidPoint', ...
            'ChangeHeading','PointToCenter','SaveTime','Radius');
    end
end

clear; clc;

%% Subplot 1: Trajectory

% load('GPYawCircConData.mat');

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

% load('GPCenCircConData.mat');

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

% load('GPYawCircConData.mat');

figure();

%SaveTime(363/3*2:end) = [];
%SaveGoalPos(:,363/3*2:end) = [];
%SaveViconPos(:,363/3*2:end) = [];
%SaveCurPos(:,363/3*2:end) = [];

subplot(3,2,1);
title("Heading to Flying Direction",'FontWeight','Normal');
hold on;
plot(SaveTime,SaveGoalPos(1,:)-SaveViconPos(1,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(1,:)-SaveCurPos(1,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("x-Error [m]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.2,0.5]);
hold off;

subplot(3,2,3);
hold on;
plot(SaveTime,SaveGoalPos(2,:)-SaveViconPos(2,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(2,:)-SaveCurPos(2,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("y-Error [m]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.2,0.3]);
hold off;

subplot(3,2,5);
hold on;
plot(SaveTime,SaveGoalPos(3,:)-SaveViconPos(3,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(3,:)-SaveCurPos(3,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
ylabel("z-Error [m]");
xlabel("Time [s]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.1,0.2]);
hold off;

clear; clc;

%% Position Graph: Trajectory 2

% load('GPCenCircConData.mat');

subplot(3,2,2);
title("Heading to Center",'FontWeight','Normal');
hold on;
plot(SaveTime,SaveGoalPos(1,:)-SaveViconPos(1,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(1,:)-SaveCurPos(1,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.3,0.4]);
hold off;

subplot(3,2,4);
hold on;
plot(SaveTime,SaveGoalPos(2,:)-SaveViconPos(2,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(2,:)-SaveCurPos(2,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.3,0.4]);
hold off;

subplot(3,2,6);
hold on;
plot(SaveTime,SaveGoalPos(3,:)-SaveViconPos(3,:),'r','LineWidth',1);
plot(SaveTime,SaveGoalPos(3,:)-SaveCurPos(3,:),'b','LineWidth',1);
plot(SaveTime,zeros(1,size(SaveTime,2)),'k--');
xlabel("Time [s]");
xlim([SaveTime(1),SaveTime(end)]);
ylim([-0.3,0.2]);
hold off;

%% RMSE

RMSE = sqrt(sum((vecnorm(SaveGoalPos-SaveViconPos)).^2)/size(SaveGoalPos,2))
