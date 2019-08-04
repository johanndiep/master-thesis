% Johann Diep (jdiep@student.ethz.ch)- August 2019
%
% Calculates the negative log marginal likelihood for the sparse approximation.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - p: Induced pseudo-inputs in form (1 x n), noise standard deviation and scalar kernel parameters
%   - m: Number of pseudo-input data
%
% Output:
%   - NegLogLikelihood: Returns the negative log of p(Y|X)

function NegSparseLogLikelihood = getSparseLogLikelihood(X,Y,p,m)
    X_ind = p(1:m);
    NoiseStd = p(m+1);
    s0 = p(m+2); s1 = p(m+3);

    Knm = PeriodicKernel(X,X_ind,s0,s1);
    Kmm = PeriodicKernel(X_ind,X_ind,s0,s1)+1e-6*eye(size(X_ind,2));
    Knn = PeriodicKernel(X,X,s0,s1);
    
    U = chol(Kmm);
    a = U\(U'\Knm');
    lambda = diag(diag(Knn-Knm*a));
    
    A = Knm*a+lambda+NoiseStd^2*eye(size(X,2));
    V = chol(A);
    b = V\(V'\Y');
    
    NegSparseLogLikelihood = -1*(-0.5*Y*b-sum(log(diag(V)))-0.5*size(X,2)*log(2*pi));
end