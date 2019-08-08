% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Executes Sparse Gaussian Process prediction.
%
% Input:
%   - X: Data parameter in form (d x n)
%   - Y: Response parameter in form (1 x n)
%   - Xt: Testing data in form (d x n)
%   - Kernel: Corresponding kernel function handle
%   - Xi: Pseudo-input data in form (d x n)
%   - NoiseStd: Noise standard deviation
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - LogLikelihood: Returns the negative log of p(Y|X,Xi,hyperparameters)

function [Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel,Xi,NoiseStd,s0,s1,s2)
    if nargin == 8
        s2 = 1;
    end    

    Model = SparseGaussianModel(X,Y,Kernel,Xi,NoiseStd^2,s0,s1,s2);
    [Mean,Covariance,LogLikelihood] = SparseGaussianPrediction(Model,Xt,Kernel);
end