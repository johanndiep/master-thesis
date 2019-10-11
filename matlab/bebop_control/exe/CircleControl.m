% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script controls the state of the Bebop towards a desired goal 
% position and velocity. Thereby, the goal position is changing per
% iteration in a circular manner, resulting in the drone following 
% a circular trajectory. Similiar to "PositionControl.m", it takes the 
% feedback from the VICON positioning system, pass it through a 
% constant velocity modeled EKF in order to estimate the position and velocity 
% and uses a PD controller to move the drone towards the goal state.
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
%     [Yes, VICON readings and iterations occur at high frequency. Sometimes 
%     drone stops but does not land, which require an additional command send
%     over terminal.]
%   - Tune the time-variant goal velocities and goal state rate 
%     such that the flight is smooth. Is goal velocities and goal state rate 
%     coupled?
%     [An equation is relating frequency to absolute velocities now.]
%   - Is the dT timing accurate?
%
% Step-by-Step:
%   1. Calibrate the VICON system and place the origin in the middle of the room 
%      with the T-link.
%   2. Attach VICON markers on the Bebop, group the markers on the VICON
%      Tracker to an object and name it "Bebop_Johann".
%   3. Place the drone such that the body-fixed frame (x-forward,y-left,z-ascend)
%      is aligned with the VICON frame.
%   4. Connect the computer with the VICON machine via Ethernet.
%   5. Turn on the Bebop and connect the laptop with it over Wi-Fi.
%   6. Turn on the Spacemouse and start its ROS driver.
%   7. Start the ROS VICON bridge node.
%   8. Start the ROS driver for the Bebop.
%   9. Set the desired circle parameters.
%   10. Run the following script.

clear; clc;

rosshutdown; rosinit;

%% Parameters

MidPoint = [0,0];
Height = 1;
Radius = 1;
Frequency = 1/30;
AbsVel = 2*Radius*pi*Frequency;

% initialize the trajectory object
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

ChangeHeading = false; % drone is pointing in the direction of flight

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
    
    % prior update with process model
    dT = toc; Time = Time + dT;
    Model.UpdatePrior(dT);
    
    % Vicon ground-truth position and quaternion of the drone
    [ViconPos,ViconQuat] = getGroundTruth(VicDroneSub);
    
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

if ChangeHeading == false
    save('VicCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos');
else
    save('VicYawCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos');    
end
    
%% Plotting and Results

if ChangeHeading == false
    load('VicCircConData.mat');
else
    load('VicYawCircConData.mat');
end

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