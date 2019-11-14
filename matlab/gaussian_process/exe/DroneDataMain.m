% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script executes the standard Gaussian Process prediction as explained
% in "Gaussian Process for Machine Learning" by Carl Rasmussen and Christopher 
% Williams for the tag-yaw-at-constant-distance experiment dataset. The 
% underlying function is approximated with a  mean and variance for each 
% testing input given the data. 
%
% In order to optimize the performance, the following parameters need to 
% be tuned:
%   - Initialization parameters
%
% Furthermore, the following points need to be investigated:
%   - Are there better optimization methods in Matlab? 
%     [Using GPy if computation takes too long.]

clear; clc;

load('RotationalMeasurements.mat'); % sample measurements

%% Data Preprocessing

DataPrepObj = DataPrep(RangeArray);
[X,Y] = DataPrepObj.ConstDistanceYaw(DroneQuaternionGroundTruthArray,2);

X = X';
Y = Y';

%% Parameters

s0 = 1; s1 = 1; NoiseStd = 1; % kernel parameters initialization
Xt = linspace(-pi,pi,2000); % testing data
Kernel = @PeriodicKernel;

%% Optimization

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getLogLikelihood(X,Y,Kernel,p(1),p(2),p(3)); 

s = fmincon(LogLikelihood,[NoiseStd,s0,s1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,s(1),s(2),s(3));

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(X,Y,'ko','MarkerSize',2);
xlabel('Yaw Angle [Radian]');
ylabel('Ranging Offset [m]');

legend('Double Standard Deviations','Mean Prediction','Training Data', ...
    'Location','northeast');

grid on;
hold off;

disp("Kernel: PeriodicKernel")
disp("Training time: "+time+" seconds");
disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
disp("Number of training points: "+size(X,2));
disp("Estimated noise standard deviation: "+s(1));
disp("Kernel hyperparameters: "+s(2)+"/"+s(3));