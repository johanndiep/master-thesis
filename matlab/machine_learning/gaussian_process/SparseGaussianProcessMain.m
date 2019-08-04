% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script executes the sparse Gaussian Process prediction described in
% "Sparse Gaussian Processes using Pseudo-inputs" by Edward Snelson and 
% Zoubin Ghahramani.

clear;
clc;

%% Parameters

f = @(x) sin(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters
NoiseStd = 0.1; % standard deviation for noise
a = -pi; b = pi; % interval of training data [a,b]
X = a + (b-a).*rand(1,100); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
m = 15; % number of pseudo-input data
X_ind = linspace(-pi,pi,m); % pseudo-input data

%% Optimization

NegSparseLogLikelihood = @(p) getSparseLogLikelihood(X,Y,p,m); % negative log marginal likelihood as objective function

options = optimoptions('fmincon','Display','iter','Algorithm','active-set');
s = fmincon(NegSparseLogLikelihood,[X_ind,NoiseStd,s0,s1],[],[],[],[],...
    [-pi*ones(1,m),0,0,0],[pi*ones(1,m),10,10,10],[],options);