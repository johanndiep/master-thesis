% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the partial derivatives of the pose periodic
% kernel function ("PosePerKernel.m") as described in "GP-BayesFilters: 
% Bayesian Filtering Using Gaussian Process Prediction and Observation 
% Models" by Jonathan Ko and Dieter Fox.
%
% Input:
%   - Xt: Testing data in form (3 x 1)
%   - Xi: Pseudo-input data in form (3 x m)
%   - s0/s1: Scalar kernel parameters
%   - s2: Scalar kernel parameter for the PoseKernel, ignored here

function KernDer = PosePerKernDeriv(Xt,Xi,s0,s1,s2)
    S1 = atan2(Xt(2),Xt(1));
    S2 = atan2(Xi(2,:),Xi(1,:));

    K = s0*exp(-0.5*(sin(0.5*bsxfun(@minus,S1',S2))/s1).^2);
    L = -sin(0.5*bsxfun(@minus,S1',S2))/s1^2;
    M = cos(0.5*bsxfun(@minus,S1',S2));
   
    Nx = -0.5*Xt(2)/norm(Xt(1:2))^2;
    Ny = 0.5*Xt(1)/norm(Xt(1:2))^2;

    A = K'.*(L'.*M');
    
    Dx = Nx*A;
    Dy = Ny*A;
    V = zeros(size(Xi,2),1);
    
    KernDer = [Dx,V,Dy,V,V,V];
    
end