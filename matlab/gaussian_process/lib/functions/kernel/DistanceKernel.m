% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% This function calculates the distance kernel matrix.
%
% Input:
%   - r1: Data matrix in form (3 x nx)
%   - r2: Data matrix in form (3 x ny)
%   - s0/s1: Scalar kernel parameters
%   - s2: Scalar kernel parameter for the PoseKernel, ignored here
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = DistanceKernel(r1,r2,s0,s1,s2)    
    nr1 = vecnorm(r1);
    nr2 = vecnorm(r2);

    A = bsxfun(@minus,nr1',nr2).^2/s1;
    
    K = s0*exp(-0.5*A);
end