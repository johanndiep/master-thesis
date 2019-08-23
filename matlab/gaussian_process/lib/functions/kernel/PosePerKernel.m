% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function calculates the periodic kernel matrix.
%
% Input:
%   - X1: Data matrix in form (3 x nx)
%   - X2: Data matrix in form (3 x ny)
%   - s0/s1: Scalar kernel parameters
%   - s2: Scalar kernel parameter for the PoseKernel, ignored here
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = PosePerKernel(X1,X2,s0,s1,s2)
    S1 = atan2(X1(2,:),X1(1,:));
    S2 = atan2(X2(2,:),X2(1,:));

    K = s0*exp(-0.5*(sin(0.5*bsxfun(@minus,S1',S2))/s1).^2);
end