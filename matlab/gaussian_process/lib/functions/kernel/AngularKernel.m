% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the pose kernel matrix. For more informations,
% check out https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/229319/3/cctaconf17.pdf.
%
% Input:
%   - r1: Data matrix in form (3 x nx)
%   - r2: Data matrix in form (3 x ny)
%   - s0/s1: Scalar kernel parameters
%   - s2: Scalar kernel parameter for the PoseKernel, ignored here
%
% Output:
%   - K: covariance matrix in the form (nx x ny)

function K = AngularKernel(r1,r2,s0,s1,s2)
    DotProduct = r1'*r2; 
    
    nr1 = vecnorm(r1);
    nr2  =vecnorm(r2);

    A = (1-DotProduct./bsxfun(@times,nr1',nr2))/s1;
    
    K = s0*exp(-A);
end