% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Set the desired parameters.
%
% Output:
%   - NumberOfIterations: Number of desired waypoints
%   - NumberOfIterationsForRanging: Desired amount of ranges to be gathered before averaged
%   - NumberOfIterationsForCalibration: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup
%   - AnchorMarkers: Coordinates of the anchor markers in anchor system body-frame
%   - TagMarker: Coordinates of the tag marker in Bebop body-frame
%   - TargetPosition: Goal position for PID control

function [NumberOfIterations,NumberOfIterationsForRanging,NumberOfIterationsForCalibration,NumberOfAnchors,AnchorMarkers,TagMarker,TargetPosition] = DesiredParameters()
    NumberOfAnchors = 6;
    
    NumberOfIterations = 2000;
    NumberOfIterationsForCalibration = 100; 
    NumberOfIterationsForRanging = 10;
    
    AnchorMarkers = [-1081.34,-1251.69,27.1459;-1081.34,-1251.69,27.1459;2207.88,-1246.52,-67.3247;2207.88,-1246.52,-67.3247;-1126.54,2498.21,40.1788;-1126.54,2498.21,40.1788]/1000; % need to be changed if the anchors are moved
    TagMarker = [12.8734,7.11919,51.7936]/1000; % need to be changed if markers are moved
    
    TargetPosition = [2;2;1];
end