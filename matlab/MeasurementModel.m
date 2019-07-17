% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function returns the measurement model for the Kalman Filter.
%
% Input:
%   - x: State
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]
%
% Output:
%   - h: Measurement model

function h = MeasurementModel(x,AnchorPositions,ZeroMeasurements)
    IterationIndex = 1;

    OffsetValue = [0.1,0.1,0.1,0.1,0.1,0.1];    
    % non-linear measurement prediction model including ellipsoidal offset assumption
    h = sym(zeros(size(AnchorPositions,1)-size(ZeroMeasurements,1),1));
    for i = 1:size(AnchorPositions,1)
        if ~ismember(i,ZeroMeasurements)
            h(IterationIndex) = sqrt((x(1)-AnchorPositions(i,1))^2+(x(2)-AnchorPositions(i,2))^2+(x(3)-AnchorPositions(i,3))^2) + OffsetValue(i);
            IterationIndex = IterationIndex + 1;
        end
    end
end 