% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script executes the standard Gaussian Process prediction for the 
% rotational dataset.

clear;
clc;

load('RotationalMeasurements.mat'); % sample measurements

%% Data Preprocessing

ErrorArray = 2-RangeArray/1000; % calculating error offset
Y = ErrorArray;

% quaternion to euler angle mapping
for i = 1:size(ErrorArray,2)
    X(i) = atan2(2*(DroneQuaternionGroundTruthArray(1,i)*DroneQuaternionGroundTruthArray(4,i)+ ...
        DroneQuaternionGroundTruthArray(2,i)*DroneQuaternionGroundTruthArray(3,i)), ...
        (1-2*(DroneQuaternionGroundTruthArray(3,i)^2+DroneQuaternionGroundTruthArray(4,i)^2)));
end

%% Parameters

s0 = 1; s1 = 1; NoiseStd = 1; % kernel and noise parameters initialization
Xt = linspace(-pi,pi,2000); % testing data
Kernel = @PeriodicKernel; % options: PeriodicKernel/PoseKernel

%% Optimization

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getLogLikelihood(X,Y,Kernel,p(1),p(2),p(3)); 

tic;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
s = fmincon(LogLikelihood,[NoiseStd,s0,s1],[],[],[],[],[0,0,0],[],[],options);
time = toc;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,s(1),s(2),s(3));

%% Plotting

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(X,Y,'ko','MarkerSize',3);
legend('Double Standard Deviations','Mean Prediction','Training Data', ...
    'Location','northeast');
txt = {"Kernel: PeriodicKernel","Training time: " + time + " seconds", ...
    "Final negative log marginal likelihood: " + LogLikelihood, ...
    "Number of training points: " + size(X,2)};
text(-2.9,0,txt)
grid on;
hold off;