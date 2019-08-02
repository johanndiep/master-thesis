% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function calculates the periodic kernel matrix.
%
% Input:
%   - X1: Data matrix in form (1 x nx)
%   - X2: Data matrix in form (1 x ny)
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = PeriodicKernel(X1,X2,s0,s1)
    K = s0*exp(-0.5*(sin(0.5*bsxfun(@minus,X1',X2))/s1).^2);
end