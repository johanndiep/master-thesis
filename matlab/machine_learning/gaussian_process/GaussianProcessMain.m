% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes the standard Gaussian Process prediction.

clear;
clc;

%% Parameters

f = @(x) sin(x); % ground-truth underlying function
s0 = 1; s1 = 1; % kernel parameters
NoiseStd = 0.1; % standard deviation for noise
a = 0; b = 2*pi; % interval of training data [a,b]
X = a + (b-a).*rand(1,2000); % training data
Y = f(X)+normrnd(0,NoiseStd,[1,size(X,2)]); % response data
Xt = linspace(0,2*pi,2000); % testing data

%% Optimization

NegLogLikelihood = @(p) getLogLikelihood(X,Y,p(1),p(2),p(3)); % negative log marginal likelihood as objective function

options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
s = fmincon(NegLogLikelihood,[1,s0,s1],[],[],[],[],[0,0,0],[100,100,100],[],options);

%% Gaussian Process

[Mean,Covariance,NegLogLikelihood] = GaussianProcess(X,Y,Xt,s(1),s(2),s(3)); % training and prediction
disp("Final negative log marginal likelihood: " + NegLogLikelihood);

%% Plotting

plotCurveBar(Xt,Mean,cov2corr(Covariance));
hold on;
plot(Xt,f(Xt),'b--');
plot(X,Y,'ko','MarkerSize',3);
legend('Standard Deviation','Prediction','Ground-Truth: y=sin(x^2)',...
    'Training Data','Location','northwest');
grid on;
hold off;