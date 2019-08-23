% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the partial derivatives of the kernel function 
% as described in "GP-BayesFilters: Bayesian Filtering Using Gaussian
% Process Prediction and Observation Models" by Jonathan Ko and Dieter Fox.
% For simplcity reason, this function only works for the pose periodic
% kernel function ("PosePerKernel.m"). An update which accommodate for all
% kernel functions will follow soon.
%
% Input:
%   - Xt: Testing data in form (3 x 1)
%   - Xi: Pseudo-input data in form (3 x n)
%   - s0/s1: Scalar kernel parameters

function KernDer = KernelDeriv(Xt,Xi,s0,s1)
    S1 = atan2(Xt(2,:),Xt(1,:));
    S2 = atan2(Xi(2,:),Xi(1,:));

    L = s0*exp(-0.5*(sin(0.5*bsxfun(@minus,S1',S2))/s1).^2);
    M = -1*sin(0.5*bsxfun(@minus,S1',S2))/s1';
    
    Nx = 0.5*Xt(1)/(Xt(1)^2+Xt(2)^2);
    Ny = 0.5*Xt(2)/(Xt(1)^2+Xt(2)^2);

    Dx = (-1)*L.*M*Nx;
    Dy = L.*M*Ny;
    V = zeros(size(Xi,2),1);
    
    
    KernDer = [Dx',V,Dy',V,V,V];
    
end