% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Calculates the negative log marginal likelihood.
%
% Input:
%   - X: Data parameter in form (d x n)
%   - Y: Response parameter in form (1 x n)
%   - NoiseStd: Noise standard deviation
%   - Kernel: Corresponding kernel function handle
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - LogLikelihood: Returns the negative log of p(Y|X,hyperparameters)

function LogLikelihood = getLogLikelihood(X,Y,Kernel,NoiseStd,s0,s1,s2)
    if nargin == 6
        s2 = 1;
    end

    K = Kernel(X,X,s0,s1,s2);
    Kn = K+NoiseStd^2*eye(size(X,2));
    cKn = chol(Kn);
    
    LogLikelihood = -1*(-0.5*Y*(Kn\Y')-sum(log(diag(cKn)))-0.5*size(X,2)*log(2*pi));
end