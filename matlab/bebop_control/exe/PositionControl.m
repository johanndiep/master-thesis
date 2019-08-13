% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script controls the position of the drone towards a desired goal position.

clear;
clc;

rosinit;

%% Parameters

GoalPos = [2,2,2];
GoalVel = [0,0,0];
dT = 0.1;

%% Preliminary

JoySubscriber = rossubscriber('/spacenav/joy');
ViconDroneSubscriber = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');
ControlObject = Controller();

%% PID

JoyMessage = JoySubscriber.LatestMessage;
while JoyMessage.Buttons(1) == 0
   JoyMessage = JoySubscriber.LatestMessage; 
end

ControlObject.Start;

Model = ConstantVelocityEKF(dT);

while true
    JoyMessage = JoySubscriber.LatestMessage;
    if JoyMessage.Buttons(2) == 1
        break;
    end
    
    [ViconPos,ViconQuat] = getGroundTruth(ViconDroneSubscriber);
    Model.UpdatePrior;
    [CurPos,CurVel] = Model.UpdateMeasurement(ViconPos);
    
    ControlObject.NoTurnFlight(CurPos,GoalPos,CurVel,GoalVel,ViconQuat);
end

ControlObject.End;

pause(2);
rosshutdown;
