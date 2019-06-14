% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This functions preprocess some data for the VanillaEKF in order
% to save computational effort, which includes setting up the parameters,
% building the non-linear objective function as well as calculating 
% its Jacobian.
%
% Input:
%   - AnchorPositions: The position coordinates of each anchor in format [NumberofAnchors,3]
% Output:
%   - h: non-linear measurement prediction model
%   - H: Jacobian of non-linear measurement prediction model

function [h,H] = PreprocessingVanillaEKF(AnchorPositions)
    IterationIndex = 1;

    % position and velocity parameters
    syms p_x p_y p_z
    syms v_x v_y v_z
    x = [p_x,p_y,p_z,v_x,v_y,v_z];
    
    % non-linear measurement prediction model
    h = sym(zeros(size(AnchorPositions,1),1));
    for i = 1:size(AnchorPositions,1)
        h(IterationIndex) = sqrt((x(1)-AnchorPositions(i,1))^2+(x(2)-AnchorPositions(i,2))^2+(x(3)-AnchorPositions(i,3))^2);
        IterationIndex = IterationIndex + 1;
    end
    
    % measurement matrix
    H = jacobian(h,x);
    
    % allow function to handle array input
    H = matlabFunction(H);
    H = convertToAcceptArray(H);
    h = matlabFunction(h);
    h = convertToAcceptArray(h);  
end