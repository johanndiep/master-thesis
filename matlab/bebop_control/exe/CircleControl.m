% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script controls the state of the Bebop towards a desired goal 
% position and velocity. Thereby, the goal position is changing per
% iteration in a circular manner, resulting in the drone following 
% a circular trajectory. Similiar to "PositionControl.m", it takes the 
% feedback from the VICON positioning system, pass it through a 
% constant velocity modeled EKF in order to estimate the velocity 
% and uses a PD controller to move the drone towards the goal state.
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
%   - Tune the time-variant goal velocities and goal state rate 
%     such that the flight is smooth.
%
% Step-by-Step:
%   1. Calibrate the VICON system and place the origin in the room with the
%      T-link, here the T-link should be placed in the middle of the room
%   2. Attach VICON markers on the Bebop, group the markers on the VICON
%      Tracker to an object and name it "Bebop_Johann"
%   3. Place the drone such that the body-fixed frame (x-forward,y-left,z-ascend)
%      is aligned with the VICON frame
%   4. Connect the computer with the VICON machine via Ethernet
%   5. Turn on the Bebop and connect the laptop with it over Wi-Fi
%   6. Start the ROS driver for the Spacemouse and turn it on
%   7. Start the ROS VICON bridge node
%   8. Start the ROS driver for the Bebop
%   9. Set the desired circle parameters
%   10. Run the following script

clear;
clc;

rosshutdown; rosinit;

%% Parameters

% initialize the trajectory object
MidPoint = [0,0];
Height = 1;
AbsVel = 0.2;
Radius = 1;
Frequency = 0.01;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

%% Preliminary

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller();

% pre-allocation
SaveViconPos = zeros(3,100000);
SaveViconQuat = zeros(4,100000);
SaveCurPos = zeros(3,100000);
SaveCurVel = zeros(3,100000);

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

save('VicCircConData.mat','SaveViconPos','SaveViconQuat', ...
    'SaveCurPos','SaveCurVel','SaveGoalPos');

clear; clc;

%% Plotting and Results

load('VicCircConData.mat');

SaveCurPos = SaveCurPos(:,1:300:end);
SaveCurVel = SaveCurVel(:,1:300:end);
SaveGoalPos = SavelGoalPos(:,1:300:end);

figure();
hold on;
title("Bebop Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([-3,3]);
ylim([-3,3]);
zlim([0,2.5]);
grid on;
scatter3(SaveViconPos(1,1),SaveViconPos(2,1),SaveViconPos(3,1),200,'ro');
scatter3(SaveGoalPos(1,:),SaveGoalPos(2,:),SaveGoalPos(3,:),50,'b.');
scatter3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:),10,'ko');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    SaveCurVel(1,:),SaveCurVel(2,:),SaveCurVel(3,:),0.5,'r');
legend('Start Position','Goal Trajectory','EKF Position Estimation', ...
    'EKF Velocity Estimation');
hold off;
