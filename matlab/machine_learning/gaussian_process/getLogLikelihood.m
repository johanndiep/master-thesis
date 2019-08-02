% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Calculates only the negative log marginal likelihood.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - NoiseStd: Noise standard deviation
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function NegLogLikelihood = getLogLikelihood(X,Y,NoiseStd,s0,s1)
    Model = GaussianModel(X,Y,NoiseStd^2,s0,s1);
   
    U = Model.U;
    a = Model.a;
    
    NegLogLikelihood = -1*(-0.5*Y*a-sum(log(diag(U)))-0.5*size(X,2)*log(2*pi));
end