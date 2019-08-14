% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script controls the position of the drone towards a desired goal position.
% It takes the feedback from the VICON positioning system, pass it through
% a constant velocity modeled EKF in order to estimate the velocity and
% uses a PD controller to move the drone towards the goal position. In
% order to optimize the performance, the following parameters need to be
% tuned:
%   - P/D-gain in "Controller.m"
%   - Threshold for maximal rotation in "Controller.m"
%   - Time interval between each EKF iteration
%   - x/P-initialization in "ConstantVelocityEKF.m"
%   - R/Q-covariance in "ConstantVelocityEKF.m"
%   - Furthermore, the yaw correction method could be optimized.

clear;
clc;

rosinit;

%% Parameters

GoalPos = [2;2;2]; % desired goal position
GoalVel = [0;0;0]; % zero velocity at goal position
dT = 0.1; % time interval between each EKF iteration

%% Preliminary

% ROS subscribers for Spacemouse and VICON positioning system
JoySubscriber = rossubscriber('/spacenav/joy'); 
ViconDroneSubscriber = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObject = Controller();

% pre-allocation
SaveViconPos = zeros(3,10000);
SaveViconQuat = zeros(4,10000);
SaveCurPos = zeros(3,10000);
SaveCurVel = zeros(3,10000);

%% PID

% starting the drone after the left button on the Spacemouse is pushed
JoyMessage = JoySubscriber.LatestMessage;
while JoyMessage.Buttons(1) == 0
   JoyMessage = JoySubscriber.LatestMessage; 
end

ControlObject.Start; % starting the drone

% initializing the constant velocity modeled EKF w
Model = ConstantVelocityEKF(dT);

i = 1;
while true
    % stopping estimation when the right button on the Spacemouse is pushed
    JoyMessage = JoySubscriber.LatestMessage;
    if JoyMessage.Buttons(2) == 1
        break;
    end
    
    % Vicon ground-truth position and quaternion of the drone
    [ViconPos,ViconQuat] = getGroundTruth(ViconDroneSubscriber);
    
    % prior and posterior update with process and measurement model
    Model.UpdatePrior;
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
    
    % moving the drone towards the desired goal position while 
    % keeping the orientation fixed
    ControlObject.NoTurnFlight(CurPos,GoalPos,CurVel,GoalVel,ViconQuat);
    
    SaveViconPos(:,i) = ViconPos;
    SaveViconQuat(:,i) = ViconQuat;
    SaveCurPos(:,i) = CurPos;
    SaveCurVel(:,i) = CurVel;
end

ControlObject.End; % landing the drone

pause(2);
rosshutdown;

%% Plotting and Results

CuttingIndex = find(SaveViconPos(1,:),1,'last')+1;
SaveViconPos(:,CuttingIndex:end) = [];
SaveViconQuat(:,CuttingIndex:end) = [];
SaveCurPos(:,CuttingIndex:end) = [];
SaveCurVel(:,CuttingIndex:end) = [];

figure();
hold on;
title("Tinamu Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
grid on;

scatter3(SaveViconPos(1,1),SaveViconPos(2,1),SaveViconPos(3,1),'ko');
scatter3(GoalPos(1),GoalPos(2),GoalPos(3),'ro');
scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:),'k.');
scatter3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:),'b.');
legend('Start Position','Goal Position','Vicon Measurement', ...
    'Constant Velocity EKF Estimation');
hold off;

save('VicPosConData.mat','SaveViconPos','SaveViconQuat','SaveCurPos','SaveCurVel');