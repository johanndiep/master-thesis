% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Standard Gaussian Process is prohibitive for large data sets. This script
% executes the sparse Gaussian Process prediction described in "Sparse 
% Gaussian Processes using Pseudo-inputs" by Edward Snelson and Zoubin 
% Ghahramani. An arbitrary function is defined as a function handle. 
% Data then can be generated which can include a noise term. The underlying 
% function is then approximated with a mean and variance for each testing 
% input given the data. 

clear; clc;

warning off;

%% Parameters

f = @(x) sin(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters initialization
NoiseStd = 0.5; % standard deviation for noise
a = -pi; b = pi; % interval of training data [a,b]
t = 1000; % number of training data
X = a + (b-a).*rand(1,t); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(a,b,2000); % testing data
Kernel = @PeriodicKernel; % options: PeriodicKernel/PoseKernel

% generate pseudo-inputs
m = 15;
[~,I] = sort(rand(1,size(X,2)));
I = I(1:m);
Xi = X(1,I);

%% Optimization

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% pre-optimizing the noise and kernel parameters
PreLogLikelihood = @(t) getLogLikelihood(X,Y,Kernel,t(1),t(2),t(3));
u = fmincon(PreLogLikelihood,[1,s0,s1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getSparseLogLikelihood(X,Y,Kernel,p,u(1),u(2),u(3));

s = fmincon(LogLikelihood,Xi,[],[],[],[],a*ones(1,m),b*ones(1,m),[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel, ...
    s,u(1),u(2),u(3));

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b--');
plot(X,Y,'ko','MarkerSize',3);
for i = 1:m
   xline(s(i),':r','LineWidth',0.5);
end

legend('Standard Deviation','Prediction','Ground-Truth: y=sin(x)',...
    'Training Data','Pseudo-input locations','Location','northeast');

grid on;
hold off;

disp("Kernel: PeriodicKernel")
disp("Training time: "+time+" seconds");
disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
disp("Number of training points: "+t);
disp("Number of pseudo-input points: "+m);
disp("Estimated noise standard deviation: "+u(1));
disp("Kernel hyperparameters: "+u(2)+"/"+u(3));