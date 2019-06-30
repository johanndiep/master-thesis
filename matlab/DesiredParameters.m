% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Set the desired parameters.
%
% Output:
%   - NumberOfIterations: Number of desired waypoints
%   - NumberOfIterationsForRanging: Desired amount of ranges to be gathered before averaged
%   - NumberOfIterationsForCalibration: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup

function [NumberOfIterations,NumberOfIterationsForRanging,NumberOfIterationsForCalibration,NumberOfAnchors] = DesiredParameters()
    NumberOfAnchors = 6;
    NumberOfIterations = 1000;
    NumberOfIterationsForCalibration = 10; 
    NumberOfIterationsForRanging = 10;
end