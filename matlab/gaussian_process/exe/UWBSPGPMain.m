% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Standard Gaussian Process is prohibitive for large data sets. This 
% script executes the sparse Gaussian Process prediction described in
% "Sparse Gaussian Processes using Pseudo-inputs" by Edward Snelson and 
% Zoubin Ghahramani for the circular flight experiment dataset. 
% The underlying function is approximated with a mean and variance for 
% each testing input given the data. The hyperparameter learning part is
% split into learning the noise as well as the kernel parameters from the
% complete log marginal likelihood and the induced pseudo-inputs from the 
% sparse log marginal likelihood.
%
% Furthermore, the following points need to be investigated:
%   - It seems like convergence can not be achieved with arbitrary number
%     of pseudo-inputs as well as hyperparameter initialization. The
%     Cholesky decomposition can not be executed due to positive semi-
%     definite property. See if this problem still occurs in GPy framework.

clear; clc;

warning off;

load('UWBGroundTruthMeas.mat'); % UWB measurements

%% Data Preprocessing

DataPrepObj = DataPrep(SaveRangeArr);
[X,Y] = DataPrepObj.CircularFlight(SaveViconPos);

% save('Dataset.mat','X','Y');

%% Parameters

AnchorNr = 1; % anchor 1-6
s0 = 1; s1 = 1; NoiseStd = 1; % kernel and noise parameters initialization
Xt = linspace(-pi,pi,2000); % testing data
Kernel = @PeriodicKernel;

% generate pseudo-inputs
m = 20;
[~,I] = sort(rand(1,size(X,2)));
I = I(1:m);
Xi = X(1,I);

%% Optimization

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% pre-computing the noise and kernel parameters
PreLogLikelihood = @(t) getLogLikelihood(X,Y(AnchorNr,:),Kernel,t(1),t(2),t(3));
u = fmincon(PreLogLikelihood,[NoiseStd,s0,s1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getSparseLogLikelihood(X,Y(AnchorNr,:),Kernel,p,u(1),u(2),u(3)); 

s = fmincon(LogLikelihood,Xi,[],[],[],[],-pi*ones(1,m),pi*ones(1,m),[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y(AnchorNr,:),Xt,Kernel, ...
    s,u(1),u(2),u(3));

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(X,Y(AnchorNr,:),'ko','MarkerSize',3);
for i = 1:m
   xline(s(i),':r','LineWidth',0.5);
end

legend('Double Standard Deviations','Mean Prediction','Training Data', ...
    'Pseudo-input locations','Location','northeast');

grid on;
hold off;

disp("Kernel: PeriodicKernel")
disp("Training time: "+time+" seconds");
disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
disp("Number of training points: "+size(X,2));
disp("Number of pseudo-input points: "+m);
disp("Estimated noise standard deviation: "+u(1));
disp("Kernel hyperparameters: "+u(2)+"/"+u(3));

switch AnchorNr
    case 1, save('HyperParamAnchor1','u','s');
    case 2, save('HyperParamAnchor2','u','s');
    case 3, save('HyperParamAnchor3','u','s');
    case 4, save('HyperParamAnchor4','u','s');
    case 5, save('HyperParamAnchor5','u','s');
    case 6, save('HyperParamAnchor6','u','s');
end