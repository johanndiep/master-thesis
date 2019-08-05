% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% Gives Gaussian Process prediction.
%
% Input:
%   - Model: Trained model structure
%   - Xt: Testing data in form (1 x n)
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function [Mean,Covariance,NegLogLikelihood] = GaussianPrediction(Model,Xt)
    X = Model.X;
    Y = Model.Y;
    s0 = Model.s0;
    s1 = Model.s1;
    U = Model.U;
    a = Model.a;
    NoiseVariance = Model.NoiseVariance;
    
    Kt = PeriodicKernel(X,Xt,s0,s1);
    Ktt = PeriodicKernel(Xt,Xt,s0,s1);
    b = U'\Kt;
    
    Mean = a'*Kt;
    Covariance = Ktt-b'*b+NoiseVariance*eye(size(Mean,2));
    NegLogLikelihood = -1*(-0.5*Y*a-sum(log(diag(U)))-0.5*size(X,2)*log(2*pi));
end