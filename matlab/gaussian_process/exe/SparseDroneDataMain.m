% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Standard Gaussian Process is prohibitive for large data sets. This 
% script executes the sparse Gaussian Process prediction described in
% "Sparse Gaussian Processes using Pseudo-inputs" by Edward Snelson and 
% Zoubin Ghahramani for the tag-yaw-at-constant-distance experiment dataset. 
% The underlying function is approximated with a mean and variance for 
% each testing input given the data. The hyperparameter learning part is
% split into learning the noise as well as the kernel parameters from the
% complete log marginal likelihood and the induced pseudo-inputs from the 
% sparse log marginal likelihood. In order to speed up the latter, the
% available data is downsampled.  

clear; clc;

warning off;

load('RotationalMeasurements.mat'); % sample measurements

%% Data Preprocessing

DataPrepObj = DataPrep(RangeArray);
[X,Y] = DataPrepObj.ConstDistanceYaw(DroneQuaternionGroundTruthArray,2);

% save('Dataset.mat','X','Y');

X = X';
Y = Y';

%% Parameters

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
PreLogLikelihood = @(t) getLogLikelihood(X,Y,Kernel,t(1),t(2),t(3));
u = fmincon(PreLogLikelihood,[NoiseStd,s0,s1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);

% downsampling
Yd = Y(1:3:end);
Xd = X(1:3:end);

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getSparseLogLikelihood(Xd,Yd,Kernel,p,u(1),u(2),u(3)); 

s = fmincon(LogLikelihood,Xi,[],[],[],[],-pi*ones(1,m),pi*ones(1,m),[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel, ...
    s,u(1),u(2),u(3));

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(X,Y,'ko','MarkerSize',3);
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