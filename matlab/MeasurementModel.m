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

function h = MeasurementModel(x,AnchorPositions)
    % parameter for offset ellipsoid in case tag facing anchor
    OffsetX = 0.2;
    OffsetY = 0.2;
    OffsetZ = 0.2;
    Numerator = OffsetX^2*OffsetY^2*OffsetZ^2; 
    
    for i = 1:size(AnchorPositions,1)
        DirectionVector = x(1:3)'-AnchorPositions(i,:)';
        DirectionVector = DirectionVector/norm(DirectionVector);
        Denominator = DirectionVector(1)^2*OffsetY^2*OffsetZ^2 + DirectionVector(2)^2*OffsetX^2*OffsetZ^2 + DirectionVector(3)^2*OffsetX^2*OffsetY^2;
        OffsetValue(i) = sqrt(Numerator/Denominator);
    end
        
    % non-linear measurement prediction model
    h = sym(zeros(size(AnchorPositions,1),1));
    for i = 1:size(AnchorPositions,1)
        h(IterationIndex) = sqrt((x(1)-AnchorPositions(i,1))^2+(x(2)-AnchorPositions(i,2))^2+(x(3)-AnchorPositions(i,3))^2) + OffsetValue(i);
        IterationIndex = IterationIndex + 1;
    end
end 