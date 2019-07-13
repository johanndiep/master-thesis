% Johann Diep (jdiep@student.ethz.ch) - May 2019
%
% This program implements the method of anchor position calibration
% described in the paper "Iterative approach for anchor configuration of
% positioning systems" by Mathias Pelka, Grigori Goronzy and Horst
% Hellbrueck.
%
% Placement of the anchors for 8 anchors network with the following assumptions:
%   - Anchor 1 is set to be the origin of the coordinate system
%   - Anchor 3, 6 and 8 are fixed on the same height as anchor 1
%   - Anchor 2, 4, 5 and 7 are fixed at a known constant height
%   - Anchor 6 is assumed to be on the same axis with anchor 1 without loss of generality
%   - Top anchors are assumed to have same x/y-coordinates as bottom anchors
%
% Placement of the anchors for 6 anchors network with the following assumptions:
%   - Anchor 1 is set to be the origin of the coordinate system
%   - Anchor 3 and 5 are fixed on the same height as anchor 1
%   - Anchor 2, 4 and 6 are fixed at a known constant height
%   - Anchor 5 is assumed to be on the same axis with anchor 1 without loss of generality
%   - Top anchors are assumed to have same x/y-coordinates as bottom anchors
%
% Input:
%   - AnchorRangeMean: Stores the averaged ranges measurements between each anchor in format [NumberOfAnchors, NumberOfAnchors]
%   - NumberOfAnchors: Amount of anchors in the setup 
%
% Output:
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]

function AnchorPositions = AnchorCalibration(AnchorRangeMean,NumberOfAnchors)    
    AnchorRangeMean = AnchorRangeMean/1000; % transform to [m] unit
    HeightTop = 2.44-0.28; % top anchors heights measured from bottom anchor
    FunctionIndex = 1; 

    for i = 1:NumberOfAnchors
        for j = 1:NumberOfAnchors
            RangesAveraged(i,j) = (AnchorRangeMean(i,j) + AnchorRangeMean(j,i))/2;
        end
    end
    
    if NumberOfAnchors == 8    
        syms a_3_x a_4_x
        syms a_2_y a_3_y a_4_y    
        AnchorPosition = [0,0,0;0,a_2_y,HeightTop;a_3_x,a_3_y,0;a_4_x,a_4_y,HeightTop;0,0,HeightTop;0,a_2_y,0;a_3_x,a_3_y,HeightTop;a_4_x,a_4_y,0];      
        a_p = [a_2_y,a_3_x,a_3_y,a_4_x,a_4_y]; % parameters to be estimated    
    elseif NumberOfAnchors == 6
        syms a_1_x
        syms a_1_y a_2_y
        AnchorPosition = [0,0,0;0,0,HeightTop;a_1_x,a_1_y,0;a_1_x,a_1_y,HeightTop;0,a_2_y,0;0,a_2_y,HeightTop];
        a_p = [a_1_x,a_1_y,a_2_y];
    end
    
    %ranging constraints
    ObjectiveFunction = sym(zeros(NumberOfAnchors*NumberOfAnchors,1));
    for i = 1:NumberOfAnchors
        for j = 1:NumberOfAnchors
            ObjectiveFunction(FunctionIndex) =  sqrt((AnchorPosition(i,1)-AnchorPosition(j,1))^2+(AnchorPosition(i,2)-AnchorPosition(j,2))^2+(AnchorPosition(i,3)-AnchorPosition(j,3))^2)-RangesAveraged(i,j);
            FunctionIndex = FunctionIndex + 1;
        end
    end
    
    Jacobian = jacobian(ObjectiveFunction,a_p); % calculate Jacobian
    
    % convert symbolic expression to function handle
    ObjectiveFunction = matlabFunction(ObjectiveFunction);
    Jacobian = matlabFunction(Jacobian);    
    
    % allow function to handle array input
    ObjectiveFunction = convertToAcceptArray(ObjectiveFunction);
    Jacobian = convertToAcceptArray(Jacobian);
    
    % working initialization from empirical analysis
    if NumberOfAnchors == 8
        a_i = [2.0349,1.9271,2.0327,1.9485,1.9104];
    elseif NumberOfAnchors == 6
        a_i = [1.8551,2.0334,2.0391];
    end
    
    while true
        EvaluatedObjectiveFunction = ObjectiveFunction(a_i); % evaluate objective function
        EvaluatedJacobian = Jacobian(a_i); % evaluate Jacobian
        IterationStep = -EvaluatedJacobian\EvaluatedObjectiveFunction; % solve linear least squares problem
        
        % update
        a_i = a_i + IterationStep';
        
        % stop iteration if norm of IterationStep passes a tolerance
        if norm(IterationStep) <= 1e-10
            break
        end
    end

    if NumberOfAnchors == 8    
        AnchorPositions = [0,0,0;0,a_i(1),HeightTop;a_i(2),a_i(3),0;a_i(4),a_i(5),HeightTop;0,0,HeightTop;0,a_i(1),0;a_i(2),a_i(3),HeightTop;a_i(4),a_i(5),0];
    elseif NumberOfAnchors == 6
        AnchorPositions = [0,0,0;0,0,HeightTop;a_i(1),a_i(2),0;a_i(1),a_i(2),HeightTop;0,a_i(3),0;0,a_i(3),HeightTop];
    end
end