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

    % parameter for offset ellipsoid in case tag facing anchor
    OffsetX = [0.0001,0.0001];
    OffsetY = [0.0001,0.0001];
    OffsetZ = [0.0001,0.0001];
    Numerator = [OffsetX(1)^2*OffsetY(1)^2*OffsetZ(1)^2,OffsetX(2)^2*OffsetY(2)^2*OffsetZ(2)^2]; 
    
    for i = 1:size(AnchorPositions,1)
        DirectionVector = x(1:3)'-AnchorPositions(i,:)';
        DirectionVector = DirectionVector/norm(DirectionVector);
        
        if mod(i,2) == 0
            Denominator = DirectionVector(1)^2*OffsetY(1)^2*OffsetZ(1)^2 + DirectionVector(2)^2*OffsetX(1)^2*OffsetZ(1)^2 + DirectionVector(3)^2*OffsetX(1)^2*OffsetY(1)^2;
            OffsetValue(i) = sqrt(Numerator(1)/Denominator);
        else
            Denominator = DirectionVector(1)^2*OffsetY(2)^2*OffsetZ(2)^2 + DirectionVector(2)^2*OffsetX(2)^2*OffsetZ(2)^2 + DirectionVector(3)^2*OffsetX(2)^2*OffsetY(2)^2;
            OffsetValue(i) = sqrt(Numerator(2)/Denominator);
        end
    end
        
    % non-linear measurement prediction model including ellipsoidal offset assumption
    h = sym(zeros(size(AnchorPositions,1)-size(ZeroMeasurements,1),1));
    for i = 1:size(AnchorPositions,1)
        if ~ismember(i,ZeroMeasurements)
            h(IterationIndex) = sqrt((x(1)-AnchorPositions(i,1))^2+(x(2)-AnchorPositions(i,2))^2+(x(3)-AnchorPositions(i,3))^2) - OffsetValue(i);
            IterationIndex = IterationIndex + 1;
        end
    end
end 