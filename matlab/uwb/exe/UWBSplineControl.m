clear; clc;

rosinit; 

load('AnchorPos.mat'); % load the anchor positions if available

%% Parameters

MarkTag = [-6.15657,6.14709,78.9883]/1000; % body-frame coordinate of tag antenna

% coordinate transformation
T = diag(ones(1,4));
T(1:3,4) = [-0.23;-0.25;0.25];
A = T*[AnchorPos';ones(1,6)]; AnchorPos = A(1:3,:)';

MidPoint = [2,1.5];
Height = 1;
Frequency = 1/40;
SplineVariable = 1;

SplinePoints = [MidPoint(1)-SplineVariable,MidPoint(2)+SplineVariable,Height; ...
    MidPoint(1)-SplineVariable,MidPoint(2)-SplineVariable,Height; ...
    MidPoint(1),MidPoint(2)-SplineVariable,Height; ...
    MidPoint(1),MidPoint(2)+SplineVariable,Height; ...
    MidPoint(1)+SplineVariable,MidPoint(2)+SplineVariable,Height; ...
    MidPoint(1)+SplineVariable,MidPoint(2)-SplineVariable,Height];

Radius = 0;
AbsVel = 0;

% initialize the trajectory object
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency,SplinePoints);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency

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
    
    % time-variant goal position and velocity
    [GoalPos,GoalYaw,GoalVel] = TrajObj.getSplinePosition(Time);
    
    % moving the drone towards the desired goal position while
    % keeping the orientation fixed
    ControlObj.TurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat,GoalYaw);

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

save('UWBVicSplineConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
        'AnchorPos','MarkTag','MidPoint','SplineVariable','SaveTime');
    
clear; clc;

%% Plot

load('UWBVicSplineConData.mat');

figure();

xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-2,MidPoint(1)+2]);
ylim([MidPoint(2)-2.5,MidPoint(2)+2.5]);
zlim([0,2.5]);
hold on;

SaveViconPos = SaveViconPos(:,1:2:end);
SaveViconQuat = SaveViconQuat(:,1:2:end);

scatter3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:)*0,1,'k.');
plot3(SaveGoalPos(1,:),SaveGoalPos(2,:),SaveGoalPos(3,:)*0,'r-');

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