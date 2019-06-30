% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Get Gauss-Newton position estimation.
%
% Input:
%   - SerialObject: Serial port object
%   - NumberOfIterationsForRanging: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]
%
% Output:
%   - GaussNewtonEstimate: Position estimate according Gauss-Newton algorithm

function GaussNewtonEstimate = getGaussNewtonEstimate(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors,AnchorPositions)
    RangeMean = getRangeMeasurement(SerialObject,NumberOfIterationsForRanging,NumberOfAnchors);
    GaussNewtonEstimate = TagPositionEstimation(AnchorPositions,RangeMean,NumberOfAnchors);
end