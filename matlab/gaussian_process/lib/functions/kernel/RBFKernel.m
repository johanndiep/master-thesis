% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the RBF kernel matrix.
%
% Input:
%   - r1: Data matrix in form (3 x nx)
%   - r2: Data matrix in form (3 x ny)
%   - s0/s1 Scalar kernel parameters
%   - s2: Scalar kernel parameter for the PoseKernel, ignored here
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = RBFKernel(r1,r2,s0,s1,s2)    
    A = (vecnorm(r1).^2)';
    B = vecnorm(r2).^2;
    C = r1'*r2;
    
    K = s0*exp(-0.5*(A-2*C+B)/s1);
end