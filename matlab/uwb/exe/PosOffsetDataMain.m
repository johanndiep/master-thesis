clear; clc;

rosinit;

load('AnchorPos.mat');

%% Parameters

% coordinate transformation
T = diag(ones(1,4));
T(1:3,4) = [-0.23;-0.25;0.25];
A = T*[AnchorPos';ones(1,6)]; AnchorPos = A(1:3,:)';

% initialize goal position and velocity
GoalPos = [2,1.5,1];
GoalVel = [0,0,0];
MidPoint = [2,1.5];
Radius = 1.5;

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
SaveViconPos = zeros(3,1000);
SaveViconQuat = zeros(4,1000);
SaveCurPos = zeros(3,1000);
SaveCurVel = zeros(3,1000);
SaveGoalPos = zeros(3,1000);
SaveGoalVel = zeros(3,1000);
SaveRangeArr = zeros(6,1000);

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
    
    % Reading UWB range measurements
    RangeArray = RangeMeasObj.TagAnchorRanging;    
    
    % prior update with process model
    dT = toc;
    Model.UpdatePrior(dT);
    
    % posterior update with measurement model
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
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

save('UWBPosConDataGP.mat','SaveViconPos','SaveViconQuat', ...
    'SaveCurPos','SaveCurVel','SaveGoalPos','SaveGoalVel','SaveRangeArr', ...
    'AnchorPos','MidPoint','Radius');

clear; clc;
