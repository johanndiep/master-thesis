% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Returns all the necessary model parameter for Sparse Gaussian Process regression.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - Xi: Pseudo-input data in form (1 x n)
%   - NoiseVariance: Noise variance
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - Model: Trained model structure with all necessary variables

function Model = SparseGaussianModel(X,Y,Xi,NoiseVariance,s0,s1)
    Kmm = PeriodicKernel(Xi,Xi,s0,s1)+1e-4*eye(size(Xi,2));
    TKmm = chol(Kmm);
    
    Knm = PeriodicKernel(X,Xi,s0,s1);
    
    lambda = diag(diag(s0*eye(size(X,2))-Knm*(TKmm\(TKmm'\Knm'))));
    
    A = lambda+NoiseVariance*eye(size(X,2));
    
    B = Knm*(TKmm\(TKmm'\Knm'))+A;
    
    Qm = Kmm+Knm'*inv(A)*Knm;
    TQm = chol(Qm);
    
    C = Knm'*inv(A)*Y';
    D = TQm\(TQm'\C);
    
    Model.Xi = Xi;
    Model.Kmm = Kmm;
    Model.B = B;
    Model.Qm = Qm;
    Model.D = D;
    Model.NoiseVariance = NoiseVariance;
    Model.s0 = s0;
    Model.s1 = s1;
    Model.Y = Y;
    Model.X = X;
end