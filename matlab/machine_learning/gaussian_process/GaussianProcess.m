% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Executes Gaussian Process prediction.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - Xt: Testing data in form (1 x n)
%   - NoiseVariance: Noise variance
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function [Mean,Covariance,NegLogLikelihood] = GaussianProcess(X,Y,Xt,NoiseStd,s0,s1)
    if nargin < 5
        [s0,s1] = deal(1); % default parameters
    end
    
    Model = GaussianModel(X,Y,NoiseStd^2,s0,s1);
    [Mean,Covariance,NegLogLikelihood] = GaussianPrediction(Model,Xt);
end