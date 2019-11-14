% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Standard Gaussian Process is prohibitive for large data sets when it comes
% to prediction speed as well as predicting local uncertainties.
% This script executes the sparse Gaussian Process prediction described in
% "Sparse Gaussian Processes using Pseudo-inputs" by Edward Snelson and 
% Zoubin Ghahramani for the tag-yaw-at-constant-distance experiment dataset. 
% The underlying function is approximated with a mean and variance for each 
% testing input given the data. The hyperparameters are pre-trained with 
% the GPy Gaussian Process framework. For more informations, check out 
% https://github.com/SheffieldML/GPy.
%
% Furthermore, the following points need to be investigated:
%   - Seems like the hyperparameters trained through GPy does not
%     accommodate for local uncertainty. 

warning off;

clear; clc;

load('RotationalMeasurements.mat'); % sample measurements
load('Hyperparameters.mat'); % pre-trained hyperparameters

%% Data Preprocessing

DataPrepObj = DataPrep(RangeArray);
[X,Y] = DataPrepObj.ConstDistanceYaw(DroneQuaternionGroundTruthArray,2);

X = X';
Y = Y';

%% Parameters

Xt = linspace(-pi,pi,2000); % testing data
Kernel = @PeriodicKernel;

%% Gaussian Process

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel, ...
    Xi,NoiseStd,s0,s1);

%% Plotting and Results

figure();

plotCurveBar(Xt,Mean,2*cov2corr(Covariance));
hold on;
plot(X,Y,'ko','MarkerSize',3);
for i = 1:size(Xi,2)
   xline(Xi(i),':r','LineWidth',0.5);
end
xlabel('Yaw Angle [Radian]');
ylabel('Ranging Offset [m]');

legend('Double Standard Deviations','Mean Prediction','Training Data', ...
    'Pseudo-input Locations','Location','northeast');

grid on;
hold off;

disp("Kernel: PeriodicKernel")
disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
disp("Number of training points: "+size(X,2));
disp("Number of pseudo-input points: "+size(Xi,2));
disp("Estimated noise standard deviation: "+NoiseStd);
disp("Kernel hyperparameters: "+s0+"/"+s1);