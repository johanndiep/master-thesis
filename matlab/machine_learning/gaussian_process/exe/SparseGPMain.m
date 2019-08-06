% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script executes the sparse Gaussian Process prediction described in
% "Sparse Gaussian Processes using Pseudo-inputs" by Edward Snelson and 
% Zoubin Ghahramani.

clear;
clc;

%% Parameters

f = @(x) sin(x).*cos(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters initialization
NoiseStd = 0.5; % standard deviation for noise
a = 0; b = 2*pi; % interval of training data [a,b]
t = 1000; % number of training data
X = a + (b-a).*rand(1,t); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(0,2*pi,2000); % testing data
Kernel = @PeriodicKernel; % options: PeriodicKernel/PoseKernel

% generate pseudo-inputs
m = 9;
[~,I] = sort(rand(1,size(X,2)));
I = I(1:m);
Xi = X(1,I);

%% Optimization

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getSparseLogLikelihood(X,Y,Kernel,p,m);

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
s = fmincon(LogLikelihood,[Xi,1,s0,s1],[],[],[],[], ...
    [a*ones(1,m),0,0,0],[b*ones(1,m),100,100,100],[],options);
time = toc;
disp("Training time: " + time + " seconds");

%% Gaussian Process

if size(s,2) == m+3
    s(m+4) = 1; % not used
end

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel, ...
    s(1:m),s(m+1),s(m+2),s(m+3),s(m+4));
disp("Final negative log marginal likelihood: " + LogLikelihood);

%% Plotting

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b--');
plot(X,Y,'ko','MarkerSize',3);
plot(s(1:m),-1*ones(1,m),'rx','MarkerSize',10);
legend('Standard Deviation','Prediction','Ground-Truth: y=exp(sin(x))',...
    'Training Data','Pseudo-input locations','Location','northeast');
grid on;
hold off;