% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes the standard Gaussian Process prediction as explained
% in "Gaussian Process for Machine Learning" by Carl Rasmussen and Christopher 
% Williams. An arbitrary function is defined as a function handle. Data then 
% can be generated which can include a noise term. The underlying function is 
% then approximated with a mean and variance for each testing input given the 
% data.
%
% In order to optimize the performance, the following parameters need to 
% be tuned:
%   - Initialization parameters
%
% Furthermore, the following points need to be investigated:
%   - Are there better optimization methods in Matlab? 
%     [Using GPy if computation takes too long.]

clear; clc;

%% Parameters

f = @(x) sin(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters initialization
NoiseStd = 0.2; % standard deviation for noise
a = -pi; b = pi; % interval of training data [a,b]
t = 100; % number of training data
X = a + (b-a)*rand(1,t); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(a,b,2000); % testing data
Kernel = @PeriodicKernel;

%% Optimization

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getLogLikelihood(X,Y,Kernel,p(1),p(2),p(3));

s = fmincon(LogLikelihood,[1,s0,s1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,s(1),s(2),s(3));

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b');
plot(X,Y,'ko','MarkerSize',3);

legend('Double Standard Deviations','Mean Prediction','Ground-Truth: y=sin(x)', ...
    'Training Data','Location','northeast');

grid on;
hold off;

disp("Kernel: PeriodicKernel")
disp("Training time: "+time+" seconds");
disp("Final negative log marginal likelihood: "+LogLikelihood);
disp("Number of training points: "+t);
disp("Estimated noise standard deviation: "+s(1));
disp("Kernel hyperparameters: "+s(2)+"/"+s(3));