% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes the standard Gaussian Process prediction.

clear;
clc;

NoiseStd = 0; % standard deviation for noise
X = [-3,-2,0]; % training points
Y = 3*sin(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response points

Model = GaussianModel(X,Y,NoiseStd^2,20,1); % model training

Xt = linspace(-pi,pi,100); % testing points
[Mean,Covariance] = GaussianPrediction(Model,Xt); % posterior mean and covariance

% plotting standard deviation prediction, mean prediction, ground-truth and training data
plotCurveBar(Xt,Mean,cov2corr(Covariance));
hold on;
plot(Xt,3*sin(Xt),'b-');
plot(X,Y,'ko','MarkerSize',10);
legend('Standard Deviation','Prediction','Ground-Truth: y=3*sin(x)',...
    'Training Data','Location','northwest');
grid on;
hold off;