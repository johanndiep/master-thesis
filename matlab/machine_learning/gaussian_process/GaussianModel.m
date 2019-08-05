% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% Returns all the necessary model parameter for Gaussian Process regression.
%
% Input:
%   - X: Data parameter in form (1 x n)
%   - Y: Response parameter in form (1 x n)
%   - NoiseVariance: Noise variance
%   - s0/s1: Scalar kernel parameters
%
% Output:
%   - Model: Trained model structure with all necessary variables

function Model = GaussianModel(X,Y,NoiseVariance,s0,s1)
    if nargin < 4
        [s0,s1] = deal(1); % default parameters
    end
    
    K = PeriodicKernel(X,X,s0,s1);
    U = chol(K+NoiseVariance*eye(size(X,2)));
    a = U\(U'\Y');
    
    Model.X = X;
    Model.Y = Y;
    Model.U = U;
    Model.a = a;
    Model.s0 = s0;
    Model.s1 = s1;
    Model.NoiseVariance = NoiseVariance;
end