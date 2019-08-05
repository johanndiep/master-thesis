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
a = 0; b = 2*pi; % interval of training data [a,b]
X = a + (b-a).*rand(1,1000); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(0,2*pi,2000); % testing data

% generate pseudo-inputs
m = 1;
[~,I] = sort(rand(1,size(X,2)));
I = I(1:m);
Xi = X(1,I);

%% Optimization

NegLogLikelihood = @(p) getSparseLogLikelihood(X,Y,p,m); % negative log marginal likelihood as objective function

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
s = fmincon(NegLogLikelihood,[Xi,NoiseStd,s0,s1],[],[],[],[], ...
    [zeros(1,m),0,0,0],[],[],options);

%% Gaussian Process

% training and prediction
[Mean,Covariance,NegLogLikelihood] = SparseGaussianProcess(X,Y,Xt, ...
    s(1:m),s(m+1),s(m+2),s(m+3));
disp("Final negative log marginal likelihood: " + NegLogLikelihood);

%% Plotting

plotCurveBar(Xt,Mean,cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b--');
plot(X,Y,'ko','MarkerSize',3);
legend('Standard Deviation','Prediction','Ground-Truth: y=sin(x)',...
    'Training Data','Location','northwest');
grid on;
hold off;