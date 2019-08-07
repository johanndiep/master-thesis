% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script executes the standard Gaussian Process prediction for the 
% rotational dataset.

warning off;

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

% downsampling
Y = Y(1:3:end);
X = X(1:3:end);

%% Parameters

s0 = 1; s1 = 1; NoiseStd = 1; % kernel and noise parameters initialization
Xt = linspace(-pi,pi,2000); % testing data
Kernel = @PeriodicKernel; % options: PeriodicKernel/PoseKernel

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
u = fmincon(PreLogLikelihood,[NoiseStd,s0,s1],[],[],[],[],[0,0,0],[],[],options);

% negative log marginal likelihood as objective function
LogLikelihood = @(p) getSparseLogLikelihood(X,Y,Kernel,p,u(1),u(2),u(3)); 

s = fmincon(LogLikelihood,Xi,[],[],[],[],-pi*ones(1,m),pi*ones(1,m),[],options);
time = toc;

%% Gaussian Process

if size(u,2) == 3
    u(4) = 1;
end

% prediction at testing data
[Mean,Covariance,LogLikelihood] = SparseGaussianProcess(X,Y,Xt,Kernel, ...
    s,u(1),u(2),u(3),u(4));

%% Plotting

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
disp("Training time: " + time + " seconds");
disp("Final negative sparse log marginal likelihood: " + LogLikelihood);
disp("Number of training points: " + size(X,2));
disp("Number of pseudo-input points: " + m);