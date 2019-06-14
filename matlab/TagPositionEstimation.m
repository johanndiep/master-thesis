% Johann Diep (jdiep@student.ethz.ch) - May 2019
%
% This function Estimates the position of the tag with a set of range 
% measurements from each anchor with Gauss-Newton optimization.
%
% Input:
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]
%   - RangeMean: Stores the range measurement towards all anchors in format [NumberOfAnchors,1]
%   - NumberOfAnchors: Amount of anchors in the setup 
%
% Output:
%   - TagPosition: Stores the position of the tag in format [3,1]

function TagPosition = TagPositionEstimation(AnchorPositions,RangeMean,NumberOfAnchors)
    RangeMean = RangeMean/1000; % transform to [m] unit 
    FunctionIndex = 1; 
    
    % coordinates of the current position of the tag
    syms p_x p_y p_z 
    p = [p_x,p_y,p_z];
    
    %ranging constraints
    ObjectiveFunction = sym(zeros(NumberOfAnchors,1));
    for i = 1:NumberOfAnchors
        ObjectiveFunction(FunctionIndex) = sqrt((p(1)-AnchorPositions(i,1))^2+(p(2)-AnchorPositions(i,2))^2+(p(3)-AnchorPositions(i,3))^2)-RangeMean(i);
        FunctionIndex = FunctionIndex + 1;
    end
        
    Jacobian = jacobian(ObjectiveFunction,p); % calculate Jacobian
    
    % convert symbolic expression to function handle
    ObjectiveFunction = matlabFunction(ObjectiveFunction);
    Jacobian = matlabFunction(Jacobian);
    
    % allow function to handle array input
    ObjectiveFunction = convertToAcceptArray(ObjectiveFunction);
    Jacobian = convertToAcceptArray(Jacobian);
    
    % initialization
    p_i = normrnd(0,0.1,[1,size(p,2)]);
        
    while true
        EvaluatedObjectiveFunction = ObjectiveFunction(p_i); % evaluate objective function
        EvaluatedJacobian = Jacobian(p_i); % evaluate Jacobian
        IterationStep = -EvaluatedJacobian\EvaluatedObjectiveFunction; % solve linear least squares problem
        
        p_i = p_i + IterationStep'; % update
        
        % stop iteration of norm of IterationStep passes a tolerance
        if norm(IterationStep) <= 1e-10
        	break
        end
    end

    TagPosition = p_i; % return solved position
end