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
%     [Yes, no limitations]
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

rosinit;

%% Parameters

MidPoint = [2,1.5];
Height = 1;
Radius = 1.25;
Frequency = 1/25;
AbsVel = 2*Radius*pi*Frequency;

% initialize the trajectory object
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = true; % fast iteration frequency
ChangeHeading = true; % able to change its direction during flight
PointToCenter = true; % able to face to the center of the circle

%% Preliminary

% ROS subscribers for Spacemouse and VICON positioning system
JoySub = rossubscriber('/spacenav/joy'); 
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

% initializing a controller object
ControlObj = Controller(FastModus);

% pre-allocation
SaveViconPos = zeros(3,30000);
SaveViconQuat = zeros(4,30000);
SaveCurPos = zeros(3,30000);
SaveCurVel = zeros(3,30000);
SaveGoalPos = zeros(3,30000);
SaveGoalVel = zeros(3,30000);
SaveTime = zero(1,30000);

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
    save('VicCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','MidPoint', ...
        'ChangeHeading','PointToCenter','Radius','SaveTime');
else
    if PointToCenter == false
        save('VicYawCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','MidPoint', ...
            'ChangeHeading','PointToCenter','Radius','SaveTime');
    else
        save('VicCenCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','MidPoint', ...
            'ChangeHeading','PointToCenter','Radius','SaveTime');
    end
end

clear; clc;

%% Subplot 1: Trajectory

load('VicYawCircConData.mat');

figure();
subplot(1,2,1);
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-Radius-1,MidPoint(1)+Radius+1]);
ylim([MidPoint(2)-Radius-1,MidPoint(2)+Radius+1]);
zlim([0,2.5]);
hold on;

SaveCurPos = SaveCurPos(:,1:120:end);
SaveViconQuat = SaveViconQuat(:,1:120:end);

scatter3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:)*0,1,'k.');

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
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Xb(1,:),Xb(2,:),Xb(3,:),0.3,'r');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Yb(1,:),Yb(2,:),Yb(3,:),0.3,'g');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Zb(1,:),Zb(2,:),Zb(3,:),0.3,'b');

grid on;
view(35.1654,48.1915)

clear; clc;

%% Subplot 2: Trajectory

load('VicCenCircConData.mat');

subplot(1,2,2);
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-Radius-1,MidPoint(1)+Radius+1]);
ylim([MidPoint(2)-Radius-1,MidPoint(2)+Radius+1]);
zlim([0,2.5]);
hold on;

SaveCurPos = SaveCurPos(:,1:120:end);
SaveViconQuat = SaveViconQuat(:,1:120:end);

scatter3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:)*0,1,'k.');

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
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Xb(1,:),Xb(2,:),Xb(3,:),0.3,'r');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Yb(1,:),Yb(2,:),Yb(3,:),0.3,'g');
quiver3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:), ...
    Zb(1,:),Zb(2,:),Zb(3,:),0.3,'b');

grid on;
view(35.1654,48.1915)

hold off;