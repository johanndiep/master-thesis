% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Calculates the negative log marginal likelihood for the sparse approximation.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - Kernel: Corresponding kernel function handle
%   - Xi: Pseudo-input data in form (1 x n)
%   - NoiseVariance: Noise variance
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - LogLikelihood: Returns the negative log of p(Y|X,Xi,hyperparameters)

function LogLikelihood = getSparseLogLikelihood(X,Y,Kernel,Xi,NoiseStd,s0,s1,s2)
    if nargin == 7
        s2 = 1;
    end
    
    Kmm = Kernel(Xi,Xi,s0,s1,s2);    
    Knm = Kernel(X,Xi,s0,s1,s2);
    
    lambda = diag(s0*ones(1,size(X,2)))-diag(sum(Knm'.*(Kmm\Knm')));
    A = lambda+NoiseStd^2*eye(size(X,2));
    
    B = Knm*(Kmm\Knm')+A;
    cB = chol(B);
    
    LogLikelihood = -1*(-0.5*Y*(B\Y')-sum(log(diag(cB)))-0.5*size(X,2)*log(2*pi));  
end