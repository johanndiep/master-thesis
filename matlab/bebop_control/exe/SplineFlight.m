% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script controls the Bebop drone to fly a predefined spline.

clear; clc; 

rosshutdown; rosinit;

%% Parameters

% initialize the trajectory object
MidPoint = [0,0];
Height = 1;
Frequency = 1/100;
SplineVariable = 0.5;
Radius = 0; % placeholder
AbsVel = 0; % placeholder
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency,SplineVariable);

Time = 0; % helper variable to estimate the time-variant goal state

%% Preliminary

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller(ChangeHeading);

% pre-allocation
SaveViconPos = zeros(3,30000);
SaveViconQuat = zeros(4,30000);
SaveCurPos = zeros(3,30000);
SaveCurVel = zeros(3,30000);
SaveGoalPos = zeros(3,30000);

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
    
    % prior and posterior update with process and measurement model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
    tic;
    
    % time-variant goal position, yaw and velocity
    [GoalPos,GoalYaw,GoalVel] = TrajObj.getYawCircleTraj(Time);
        
    % moving the drone towards the desired goal position while 
    % keeping the orientation in the direction of flight
    ControlObj.TurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat,GoalYaw);
    
    % saving to array
    SaveViconPos(:,i) = ViconPos;
    SaveViconQuat(:,i) = ViconQuat;
    SaveCurPos(:,i) = CurPos;
    SaveCurVel(:,i) = CurVel;
    SaveGoalPos(:,i) = GoalPos';
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

save('VicSplineConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos');

 %% Plotting and Results

load('VicSplineConData.mat');
    
SaveViconPos = SaveViconPos(:,1:300:end);
SaveViconQuat = SaveViconQuat(:,1:300:end);
SaveCurPos = SaveCurPos(:,1:300:end);
SaveCurVel = SaveCurVel(:,1:300:end);
SaveGoalPos = SaveGoalPos(:,1:300:end);

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
    'EKF Position Estimation','EKF Velocity Estimation');

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