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

function [NumberOfIterations,NumberOfIterationsForRanging,NumberOfIterationsForCalibration,NumberOfAnchors,AnchorMarkers,TagMarker] = DesiredParameters()
    NumberOfAnchors = 6;
    
    NumberOfIterations = 1000;
    NumberOfIterationsForCalibration = 500; 
    NumberOfIterationsForRanging = 10;
    
    AnchorMarkers = [-1060.82,-1262.19,42.6497;-1060.82,-1262.19,42.6497;2223.15,-1247.26,-88.1794;2223.15,-1247.26,-88.1794;-1162.33,2509.44,45.5296;-1162.33,2509.44,45.5296]/1000; % need to be changed if the anchors are moved
    TagMarker = [-40.4669,6.09042,68.6882]; % need to be changed if markers are moved
end