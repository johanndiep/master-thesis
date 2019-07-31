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

function [Mean,Covariance] = GaussianPrediction(Model,Xt)
    X = Model.X;
    s0 = Model.s0;
    s1 = Model.s1;
    U = Model.U;
    a = Model.a;
    
    Kt = PeriodicKernel(X,Xt,s0,s1);
    Ktt = PeriodicKernel(Xt,Xt,s0,s1);
    b = U'\Kt;
    
    Mean = a'*Kt;
    Covariance = Ktt-b'*b;
end