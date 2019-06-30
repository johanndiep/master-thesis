% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Testing the PID controller.

disp("Testing the PID controller.")
disp("******************************************************************************************************");

%% Setup

PIDObject = Controller();

IterationIndex = 1;
PreviousTime = 0;

SerialObject = SerialPortSetup();
[NumberOfIterations,NumberOfIterationsForRanging,NumberOfIterationsForCalibration,NumberOfAnchors] = DesiredParameters();

input("Place anchors in the room and press [ENTER]");
input("Change the connected module into Sniffer mode and press [ENTER]");
disp("******************************************************************************************************");
disp("Starting anchor self-calibration procedure");

AnchorPositions = getAnchorPositions(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors);
AnchorSetupPlot(AnchorPositions,NumberOfAnchors);

%% Estimation and Control

("******************************************************************************************************");
disp("Preparing to gather " + NumberOfIterations + " waypoints");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
("******************************************************************************************************");

delete(SerialObject);
SerialObject = SerialPortSetup();

PIDObject.Start;
TargetPosition = [2;2;2];

StartingPosition = getGaussNewtonEstimate(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors,AnchorPositions);

x_Posterior = [StartingPosition,normrnd(0,0.1,[1,3])]';
P_Posterior = 0.05*eye(size(x_Posterior,1));

[h,H] = PreprocessingVanillaEKF(AnchorPositions);

tic;
while IterationIndex < NumberOfIterations + 1
    z = getRangeMeasurement(SerialObject,1,NumberOfAnchors)'/1000;
    
    TimeSinceStart = toc;
    DeltaT = TimeSinceStart-PreviousTime;
    [x_Posterior,P_Posterior] = VanillaEKF(NumberOfAnchors,x_Posterior,P_Posterior,DeltaT,z,h,H);
    
    PIDObject.NoTurnFlight(x_Posterior(1:3),TargetPosition);
    
    % update
    IterationIndex = IterationIndex + 1;
    PreviousTime = TimeSinceStart;
end

PIDObject.End;