% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes the controlled position flight experiment. It reads
% VICON data over Ethernet and sends the appropriate commands to the Bebop
% drone. 

%% Execution

IterationIndex = 1;

[~,ViconDroneSubscriber] = ROSCommunication();
[NumberOfIterations,~,~,~,~,TagMarker,TargetPosition] = DesiredParameters();

ControlObject = Controller();
ControlObject.Start;
pause(2);

while IterationIndex < NumberOfIterations + 1
    [DronePositionGroundTruth,DroneQuaternionGroundTruth] = getGroundTruth(ViconDroneSubscriber);
    DroneGroundTruthPosition = getDronePosition(DronePositionGroundTruth,DroneQuaternionGroundTruth,TagMarker);    
    TrajectoryPoints(:,IterationIndex) = DroneGroundTruthPosition;
    ControlObject.NoTurnFlight(DroneGroundTruthPosition,TargetPosition);
    IterationIndex = IterationIndex + 1;
end

ControlObject.End;
pause(2);
rosshutdown;