% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Executes Sparse Gaussian Process prediction.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - Xt: Testing data in form (1 x n)
%   - Xi: Pseudo-input data in form (1 x n)
%   - NoiseVariance: Noise variance
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function [Mean,Covariance,NegLogLikelihood] = SparseGaussianProcess(X,Y,Xt,Xi,NoiseStd,s0,s1)
    Model = SparseGaussianModel(X,Y,Xi,NoiseStd^2,s0,s1);
    [Mean,Covariance,NegLogLikelihood] = SparseGaussianPrediction(Model,Xt);
end