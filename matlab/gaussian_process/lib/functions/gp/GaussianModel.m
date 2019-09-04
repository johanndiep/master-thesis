% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% Returns all the necessary model parameter for Gaussian Process regression.
%
% Input:
%   - X: Data parameter in form (d x n)
%   - Y: Response parameter in form (1 x n)
%   - Kernel: Corresponding kernel function handle
%   - NoiseVariance: Noise variance
%   - s0/s1/s2: Scalar kernel parameters
%
% Output:
%   - Model: Trained model structure with all necessary variables

function Model = GaussianModel(X,Y,Kernel,NoiseVariance,s0,s1,s2)
    if nargin == 6
        s2 = 1;
    end  

    K = Kernel(X,X,s0,s1,s2);
    Kn = K+NoiseVariance*eye(size(X,2));
    cKn = chol(Kn);
    a = Kn\Y';
    
    Model.X = X;
    Model.Y = Y;
    Model.NoiseVariance = NoiseVariance;
    Model.s0 = s0;
    Model.s1 = s1;
    Model.s2 = s2;
    Model.cKn = cKn;
    Model.a = a;
end