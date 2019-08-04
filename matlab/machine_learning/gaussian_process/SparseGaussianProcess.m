% Johann Diep (jdiep@student.ethz.ch)- August 2019

function [Mean,Covariance,NegLogLikelihood] = SparseGaussianProcess(X,Y,Xt,NoiseStd,s0,s1)
    if nargin < 5
        [s0,s1] = deal(1); % default parameters
    end
    
    Model = GaussianModel(X,Y,NoiseStd^2,s0,s1);
    [Mean,Covariance,NegLogLikelihood] = GaussianPrediction(Model,Xt);
end