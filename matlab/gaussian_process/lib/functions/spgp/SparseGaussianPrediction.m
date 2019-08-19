% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Gives Sparse Gaussian Process prediction.
%
% Input:
%   - Model: Trained model structure
%   - Xt: Testing data in form (d x n)
%   - Kernel: Corresponding kernel function handle
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - LogLikelihood: Returns the negative log of p(Y|X,Xi,hyperparameters)

function [Mean,Covariance,LogLikelihood] = SparseGaussianPrediction(Model,Xt,Kernel)
    X = Model.X;
    Y = Model.Y;
    Xi = Model.Xi;
    Kmm = Model.Kmm;
    B = Model.B;
    Q = Model.Q;
    C = Model.C;
    NoiseVariance = Model.NoiseVariance;
    s0 = Model.s0;
    s1 = Model.s1;
    s2 = Model.s2;
    
    Ktm = Kernel(Xt,Xi,s0,s1,s2);
    Ktt = Kernel(Xt,Xt,s0,s1,s2);
    
    Mean = C'*Ktm';
    Covariance = Ktt-Ktm*(Kmm\Ktm')-Ktm*(Q\Ktm')+NoiseVariance*eye(size(Xt,2));
    
    cB = chol(B);    
    
    LogLikelihood = -1*(-0.5*Y*(B\Y')-sum(log(diag(cB)))-0.5*size(X,2)*log(2*pi));
end