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

function NegLogLikelihood = getSparseLogLikelihood(X,Y,p,m)
    Xi = p(1:m); NoiseStd = p(m+1); s0 = p(m+2); s1 = p(m+3);
    
    Kmm = PeriodicKernel(Xi,Xi,s0,s1)+1e-4*eye(m);
    TKmm = chol(Kmm);
    
    Knm = PeriodicKernel(X,Xi,s0,s1);
    
    lambda = diag(diag(s0*eye(size(X,2))-Knm*(TKmm\(TKmm'\Knm'))));
    A = lambda+NoiseStd^2*eye(size(X,2));
    
    B = Knm*(TKmm\(TKmm'\Knm'))+A;
    TB = chol(B);    
    b = TB\(TB'\Y');
    
    NegLogLikelihood = -1*(-0.5*Y*b-sum(log(diag(TB)))-0.5*size(X,2)*log(2*pi));  
end