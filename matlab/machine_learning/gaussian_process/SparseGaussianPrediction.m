% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Gives Sparse Gaussian Process prediction.
%
% Input:
%   - Model: Trained model structure
%   - Xt: Testing data in form (1 x n)
%
% Output:
%   - Mean: Predicted value
%   - Covariance: Uncertainty of the predicted value
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function [Mean,Covariance,NegLogLikelihood] = SparseGaussianPrediction(Model,Xt)
    Xi = Model.Xi;
    Kmm = Model.Kmm;
    B = Model.B;
    Qm = Model.Qm;
    D = Model.D;
    NoiseVariance = Model.NoiseVariance;
    s0 = Model.s0;
    s1 = Model.s1;
    Y = Model.Y;
    X = Model.X;
    
    Ktm = PeriodicKernel(Xt,Xi,s0,s1);
    Ktt = PeriodicKernel(Xt,Xt,s0,s1);
    
    TKmm = chol(Kmm);
    TQm = chol(Qm);
    
    Mean = D'*Ktm';
    Covariance = Ktt-Ktm*(TKmm\(TKmm'\Ktm'))-Ktm*(TQm\(TQm'\Ktm'))+NoiseVariance*eye(size(Xt,2));
    
    TB = chol(B);    
    b = TB\(TB'\Y');
    
    NegLogLikelihood = -1*(-0.5*Y*b-sum(log(diag(TB)))-0.5*size(X,2)*log(2*pi));
end