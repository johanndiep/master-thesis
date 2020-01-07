clear; clc;

rosinit;

load('PositionHypGP.mat');

%% Parameters

% initialize goal position and velocity
GoalPos = [2,1.5,1];
GoalVel = [0,0,0];

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency

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
    
    % moving the drone towards the desired goal position while
    % keeping the orientation fixed
    ControlObj.NoTurnFlight(CurPos,GoalPos',CurVel,GoalVel',ViconQuat);

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

save('GPPosConData.mat','SaveViconPos','SaveViconQuat', ...
    'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel', ...
    'SaveRangeArr','Savet','SaveAbs','SaveCovVal','SaveTime');

clear; clc;
