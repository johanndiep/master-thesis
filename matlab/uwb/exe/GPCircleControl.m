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
%   - Threshold for maximal rotation in "Controller.m"
%   - Time interval between each EKF iteration
%   - x/P-initialization in "ConstantVelocityEKF.m"
%   - R/Q-covariance in "ConstantVelocityEKF.m"
%   - Goal state changing rate f in "TrajectoryGenerator.m"
%   - Absolute goal velocity in "TrajectoryGenerator.m"
%
% To-Do:
%   - Track antenna instead of object center of mass with VICON.
%   - Figure out why there are mismatch at certain areas.

clear; clc;

rosshutdown; rosinit;

load('HyperparametersGP.mat'); % load the parameters

%% Parameters

% initialize the trajectory object
MidPoint = [0,0];
Height = 1;
Radius = 1;
Frequency = 0.01;
AbsVel = 2*Radius*pi*Frequency;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

ChangeHeading = false; % drone is pointing in the direction of flight

Kernel = @RBFKernel;

%% Preliminary

% initializing a range measurement object
RangeMeasObj = RangeMeasurements();

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller(ChangeHeading);

% pre-allocation
SaveViconPos = zeros(3,5000);
SaveViconQuat = zeros(4,5000);
SaveCurPos = zeros(3,5000);
SaveCurVel = zeros(3,5000);
SaveGoalPos = zeros(3,5000);
SaveRangeArr = zeros(6,5000);
Savet = zeros(6,5000);
SaveAbs = zeros(6,5000);
SaveCovVal = zeros(6,5000);

%% PID

% starting the drone after the left button on the Spacemouse is pushed
JoyMessage = JoySub.LatestMessage;
while JoyMessage.Buttons(1) == 0
   JoyMessage = JoySub.LatestMessage; 
end

ControlObj.Start; % starting the drone
pause(5);

% initializing the constant velocity modeled EKF
Model = ConstantVelocityGP(Xa,Ya,Kernel,NoiseStd,s0,s1,AnchorPos);
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
    
    % prior and posterior update with process and measurement model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    [CurPos,CurVel,t,Abs,CovVal] = Model.UpdateMeasurement(RangeArray);
    tic;
    
    if ChangeHeading == false
        % time-variant goal position and velocity
        [GoalPos,GoalVel] = TrajObj.getCircleTrajectory(Time);
        
        % moving the drone towards the desired goal position while
        % keeping the orientation fixed
        ControlObj.NoTurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat);
    else
        % time-variant goal position, yaw and velocity
        [GoalPos,GoalYaw,GoalVel] = TrajObj.getYawCircleTraj(Time);
        
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
    SaveRangeArr(:,i) = RangeArray;
    Savet(:,i) = t;
    SaveAbs(:,i) = Abs;
    SaveCovVal(:,i) = CovVal;
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
SaveRangeArr(:,CuttingIndex:end) = [];
Savet(:,CuttingIndex:end) = [];
SaveAbs(:,CuttingIndex:end) = [];
SaveCovVal(:,CuttingIndex:end) = [];

save('GPCircConData.mat','SaveViconPos','SaveViconQuat', ...
    'SaveCurPos','SaveCurVel','SaveGoalPos','SaveRangeArr', ...
    'Savet','SaveAbs','SaveCovVal');

clear; clc;

%% Plotting and Results

load('GPCircConData.mat');

SaveViconPos = SaveViconPos(:,1:1:end);
SaveViconQuat = SaveViconQuat(:,1:1:end);
SaveCurPos = SaveCurPos(:,1:1:end);
SaveCurVel = SaveCurVel(:,1:1:end);
SaveGoalPos = SaveGoalPos(:,1:1:end);

figure();

title("Bebop Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([-2,2]);
ylim([-2,2]);
zlim([0,2.5]);
hold on;

scatter3(SaveViconPos(1,1),SaveViconPos(2,1),SaveViconPos(3,1),200,'ro');
scatter3(SaveGoalPos(1,:),SaveGoalPos(2,:),SaveGoalPos(3,:),50,'b.');
scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:),10,'r.');
scatter3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:),10,'ko');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    SaveCurVel(1,:),SaveCurVel(2,:),SaveCurVel(3,:),0.5,'k');

set(0,'DefaultLegendAutoUpdate','off')
legend('Start Position','Desired Trajectory','Vicon Position', ...
'EKF-UWB/GP Position Estimation','EKF-UWB/GP Velocity Estimation');

quiver3(0,0,0,1,0,0,0.5,'r','LineWidth',2);
quiver3(0,0,0,0,1,0,0.5,'g','LineWidth',2);
quiver3(0,0,0,0,0,1,0.5,'b','LineWidth',2);

RotMats = quat2rotm(SaveViconQuat');
Xb = permute(RotMats(:,1,:),[1,3,2]);
Yb = permute(RotMats(:,2,:),[1,3,2]);
Zb = permute(RotMats(:,3,:),[1,3,2]);
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Xb(1,:),Xb(2,:),Xb(3,:),0.2,'r');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Yb(1,:),Yb(2,:),Yb(3,:),0.2,'g');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Zb(1,:),Zb(2,:),Zb(3,:),0.2,'b');

grid on;
hold off;