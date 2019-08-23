% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Returns all the necessary model parameter for Sparse Gaussian Process regression.
%
% Input:
%   - X: Data parameter in form (d x n)
%   - Y: Response parameter in form (1 x n)
%   - Kernel: Corresponding kernel function handle
%   - Xi: Pseudo-input data in form (d x n)
%   - NoiseVariance: Noise variance
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - Model: Trained model structure with all necessary variables

function Model = SparseGaussianModel(X,Y,Kernel,Xi,NoiseVariance,s0,s1,s2)
    if nargin == 7
        s2 = 1;
    end    
    
    Kmm = Kernel(Xi,Xi,s0,s1,s2);    
    Knm = Kernel(X,Xi,s0,s1,s2);

    lambda = diag(s0*ones(1,size(X,2)))-diag(sum(Knm'.*(Kmm\Knm')));
    A = lambda+NoiseVariance*eye(size(X,2));
    B = Knm*(Kmm\Knm')+A;
    Q = Kmm+Knm'*(A\Knm);
    C = Q\(Knm'*(A\Y'));
    
    Model.X = X;
    Model.Y = Y;
    Model.Xi = Xi;
    Model.Kmm = Kmm;
    Model.B = B;
    Model.Q = Q;
    Model.C = C;
    Model.NoiseVariance = NoiseVariance;
    Model.s0 = s0;
    Model.s1 = s1;
    Model.s2 = s2;
end