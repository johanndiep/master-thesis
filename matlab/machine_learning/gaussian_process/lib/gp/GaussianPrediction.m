% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% Gives Gaussian Process prediction.
%
% Input:
%   - Model: Trained model structure
%   - Xt: Testing data in form (1 x n)
%   - Kernel: Corresponding kernel function handle
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - LogLikelihood: Returns the negative log of p(Y|X)

function [Mean,Covariance,LogLikelihood] = GaussianPrediction(Model,Xt,Kernel)
    X = Model.X;
    Y = Model.Y;
    NoiseVariance = Model.NoiseVariance;
    s0 = Model.s0;
    s1 = Model.s1;
    s2 = Model.s2;
    cK = Model.cK;
    a = Model.a;
    
    Kt = Kernel(X,Xt,s0,s1,s2);
    Ktt = Kernel(Xt,Xt,s0,s1,s2);
    b = cK'\Kt;
    
    Mean = a'*Kt;
    Covariance = Ktt-b'*b+NoiseVariance*eye(size(Xt,2));
    LogLikelihood = -1*(-0.5*Y*a-sum(log(diag(cK)))-0.5*size(X,2)*log(2*pi));
end