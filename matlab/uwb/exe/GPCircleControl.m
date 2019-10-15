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

rosshutdown; rosinit;

% load('CircleHypGP.mat'); % load the parameters
% load('YawCircleHypGP.mat'); % load the parameters with yaw
load('CenCircleHypGP.mat'); % load the center parameters 

%% Parameters

% initialize the trajectory object
MidPoint = [2.5,2];
Height = 1;
Radius = 1.5;
Frequency = 1/30;
AbsVel = 2*Radius*pi*Frequency;
TrajObj = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency);

Time = 0; % helper variable to estimate the time-variant goal state

FastModus = false; % fast iteration frequency
ChangeHeading = true; % drone pointing in direction of flight
PointToCenter = true; % able to face to the center of the circle

Kernel = @DistanceKernel;
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
SaveRangeArr = zeros(6,600);
Savet = zeros(6,600);
SaveAbs = zeros(6,600);
SaveCovVal = zeros(6,600);

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

if ChangeHeading == false
    save('GPCircConData.mat','SaveViconPos','SaveViconQuat', ...
        'SaveCurPos','SaveCurVel','SaveGoalPos','SaveRangeArr', ...
        'Savet','SaveAbs','SaveCovVal','MidPoint','ChangeHeading','PointToCenter');
else
    if PointToCenter == false
        save('GPYawCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveRangeArr', ...
            'Savet','SaveAbs','SaveCovVal','MidPoint','ChangeHeading','PointToCenter');
    else
        save('GPCenCircConData.mat','SaveViconPos','SaveViconQuat', ...
            'SaveCurPos','SaveCurVel','SaveGoalPos','SaveRangeArr', ...
            'Savet','SaveAbs','SaveCovVal','MidPoint','ChangeHeading','PointToCenter');
    end
    
end

clear; clc;

%% Plotting and Results

if ChangeHeading == false
    load('GPCircConData.mat');
else
    if PointToCenter == false
        load('GPYawCircConData.mat');
    else
        load('GPCenCircConData.mat');
    end
end

%%

figure();

title("Flight Trajectory");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
xlim([MidPoint(1)-3,MidPoint(1)+3]);
ylim([MidPoint(2)-3,MidPoint(2)+3]);
zlim([0,2.5]);
hold on;

plot3(SaveGoalPos(1,:),SaveGoalPos(2,:),SaveGoalPos(3,:),'LineWidth',0.5,'Color','b');
plot3(SaveViconPos(1,:),SaveViconPos(2,:),SaveViconPos(3,:),'LineWidth',1.5,'Color','r','LineStyle',':');
plot3(SaveCurPos(1,:),SaveCurPos(2,:),SaveCurPos(3,:),'LineWidth',1.5,'Color','k','LineStyle',':');

set(0,'DefaultLegendAutoUpdate','off')
legend('Reference','Vicon Position Measurement','EKF Position Estimation');

quiver3(0,0,0,1,0,0,0.3,'k','LineWidth',1);
quiver3(0,0,0,0,1,0,0.3,'k','LineWidth',1);
quiver3(0,0,0,0,0,1,0.3,'k','LineWidth',1);

grid on;
daspect([1 1 1]);
hold off;