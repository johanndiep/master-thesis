% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the pose kernel matrix.
%
% Input:
%   - r1: Data matrix in form (3 x nx)
%   - r2: Data matrix in form (3 x ny)
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = PoseKernel(r1,r2,s0,s1,s2)
    DotProduct = r1'*r2; 
    
    nr1 = vecnorm(r1);
    nr2  =vecnorm(r2);

    A = (1 - DotProduct./bsxfun(@times,nr1',nr2))/s1;
    B = bsxfun(@minus,nr1',nr2).^2/s2;
    
    K = s0*exp(-A-B);
end