% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function calibrates the anchor setup with the self-calibration
% procedure.
%
% Input:
%   - SerialObject: Serial port object
%   - NumberOfIterationsForCalibration: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup
%
% Output:
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]


function AnchorPositions = getAnchorPositions(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors)
    AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors);
    AnchorPositions = AnchorCalibration(AnchorRangeMean,NumberOfAnchors);
end