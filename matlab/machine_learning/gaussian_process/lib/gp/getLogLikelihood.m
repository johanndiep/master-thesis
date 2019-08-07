% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Calculates the negative log marginal likelihood.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - NoiseStd: Noise standard deviation
%   - Kernel: Corresponding kernel function handle
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - LogLikelihood: Returns the negative log of p(Y|X)

function LogLikelihood = getLogLikelihood(X,Y,Kernel,NoiseStd,s0,s1,s2)
    if nargin == 6
        s2 = 1;
    end

    K = Kernel(X,X,s0,s1,s2);
    cK = chol(K+NoiseStd^2*eye(size(X,2)));
    a = cK\(cK'\Y');
    
    LogLikelihood = -1*(-0.5*Y*a-sum(log(diag(cK)))-0.5*size(X,2)*log(2*pi));
end