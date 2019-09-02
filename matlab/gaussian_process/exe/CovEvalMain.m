% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% This script executes the standard Gaussian Process prediction as
% explained in "Gaussian Process for Machine Learning" by Carl
% Rasmussen and Christopher Williams for the circular flight experiment
% dataset. The covariance of the UWB ranging offsets are evaluated along
% the flight space.
%
% In order to optimize the performance, the following parameters need to
% be tuned:
%   - Initialization parameters
%
% Furthermore, the following points need to be investigated:
%   - Are there better optimization methods in Matlab?
%     [Using GPy if computation takes too long.]
%   - Check why the data from anchor 3 does not converge.
%   - The true ranging distance is an approximation, since the markers are
%     not planarly placed on the antennas. Does this influence the
%     prediction in any way?

clear; clc;

load('UWB-GP.mat'); % UWB and VICON measurements

%% Data Preprocessing

DataPrepObj = DataPrep(SaveRangeArr);
[X,Yd] = DataPrepObj.Flight(Marker,SaveViconPos,SaveViconQuat, ...
    VicAncPos,VicAncQuat);
    
Kernel = @PoseKernel;

% pre-allocation
NoiseStd = zeros(1,6);
s0 = zeros(1,6);
s1 = zeros(1,6);
s2 = zeros(1,6);

% testing space and testing data
a = linspace(-1.5,1.5,100);
b = linspace(-1.5,1.5,100);
[x,y] = meshgrid(a,b);
Xt(1,:) = x(:)';
Xt(2,:) = y(:)';
Xt(3,:) = ones(size(Xt(1,:)));

ShowResults = false;
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

%% Optimization

for i = 1:2
    Y = Yd(i,:);
            
    % negative log marginal likelihood as objective function
    tic;
    LogLikelihood = @(p) getLogLikelihood(X,Y,Kernel,p(1),p(2),p(3),p(4));
    s = fmincon(LogLikelihood,[1,1,1,1],[],[],[],[],[0,0,0,0],[Inf,Inf,Inf,Inf],[],options);
    time = toc;
    
    NoiseStd(i) = s(1);
    s0(i) = s(2);
    s1(i) = s(3);
    s2(i) = s(4);
        
    if ShowResults == true
        % prediction at testing data
        [Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y,Xt,Kernel,s(1),s(2),s(3),s(4));
        
        figure();

        title("Flight Space Covariance Evaluation");
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Standard Deviation [cm]");
        hold on;
        
        Std = sqrt(diag(Covariance))*100;
        scatter3(Xt(1,:),Xt(2,:),Std,10,Std);
        colormap(jet);
        colorbar;
        
        grid on;
        hold off;
        
        disp("Kernel: PoseKernel");
        disp("Training time: "+time+" seconds");
        disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
        disp("Number of training points: "+size(X,2));
        disp("Estimated noise standard deviation: "+s(1));
        disp("Kernel hyperparameters: "+s(2)+"/"+s(3)+"/"+s(4));
    end
end

save('HyperparametersGP.mat','NoiseStd','s0','s1','s2');