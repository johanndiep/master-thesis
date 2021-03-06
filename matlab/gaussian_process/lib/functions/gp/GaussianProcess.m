% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Executes Gaussian Process framework.
%
% Input:
%   - X: Data parameter in form (d x n)
%   - Y: Response parameter in form (1 x n)
%   - Xt: Testing data in form (d x n)
%   - Kernel: Corresponding kernel function handle
%   - NoiseStd: Noise standard deviation
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - LogLikelihood: Returns the negative log of p(Y|X,hyperparameters)

function [Mean,Covariance,NegLogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,NoiseStd,s0,s1,s2)
    if nargin == 7
        s2 = 1;
    end    

    Model = GaussianModel(X,Y,Kernel,NoiseStd^2,s0,s1,s2);
    [Mean,Covariance,NegLogLikelihood] = GaussianPrediction(Model,Xt,Kernel);
end