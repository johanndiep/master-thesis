% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes the standard Gaussian Process prediction.

clear;
clc;

%% Parameters

f = @(x) sin(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters initialization
NoiseStd = 0.1; % standard deviation for noise
a = 0; b = 2*pi; % interval of training data [a,b]
t = 1000; % number of training data
X = a + (b-a)*rand(1,t); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(0,2*pi,2000); % testing data
Kernel = @PeriodicKernel; % options: PeriodicKernel/PoseKernel

%% Optimization

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getLogLikelihood(X,Y,Kernel,p(1),p(2),p(3));

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
s = fmincon(LogLikelihood,[1,s0,s1],[],[],[],[],[0,0,0],[],[],options);
time = toc;
disp("Training time: " + time + " seconds");

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,s(1),s(2),s(3));
disp("Final negative log marginal likelihood: " + LogLikelihood);

%% Plotting

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b');
plot(X,Y,'ko','MarkerSize',3);
legend('Double Standard Deviations','Mean Prediction','Ground-Truth: y=sin(x)', ...
    'Training Data','Location','northeast');
grid on;
hold off;