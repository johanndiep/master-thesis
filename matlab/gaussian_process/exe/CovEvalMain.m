% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% This script executes the (Sparse) Gaussian Process prediction as
% explained in "Gaussian Process for Machine Learning" by Carl
% Rasmussen and Christopher Williams or "Sparse Gaussian Processes using
% Pseudo-inputs" by Edward Snelson and Zoubin Ghahramani for the circular 
% flight experiment dataset. The covariance of the UWB ranging offsets are 
% evaluated along the flight space.
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
%   - Influence of kernel functions.

clear; clc;

load('UWB-GP.mat'); % UWB and VICON measurements

%% Data Preprocessing

% removing zero measurements
[r,c] = find(SaveRangeArr==0);
SaveRangeArr(:,c) = [];
SaveViconPos(:,c) = [];
SaveViconQuat(:,c) = [];

% data mapping
DataPrepObj = DataPrep(SaveRangeArr);
[Xd,Xa,Yd,Ya,AnchorPos,P] = DataPrepObj.Flight(Marker,SaveViconPos,SaveViconQuat, ...
    VicAncPos,VicAncQuat);

Kernel = @RBFKernel;

% pre-allocation
NoiseStd = zeros(1,6);
s0 = zeros(1,6);
s1 = zeros(1,6);

% testing space and testing data
a = linspace(-1.5,1.5,100);
b = linspace(-1.5,1.5,100);
[x,y] = meshgrid(a,b);
Xt(1,:) = x(:)';
Xt(2,:) = y(:)';
Xt(3,:) = ones(size(Xt(1,:)));

ShowResults = true;
Save = false;
Mode = "SPGP";
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% generate pseudo-inputs
if Mode == "SPGP"
    m = 60;

    MidPoint = [0,0];
    Height = 1;
    Radius = 1;
    
    c = linspace(0,2*pi,100);
    x = MidPoint(1)+Radius*sin(c);
    y = MidPoint(2)-Radius*cos(c);
    
    RandIndex = randperm(length(c),m);
    
    Si = [x(1,RandIndex);y(1,RandIndex)];
    
    nonlcon = @(n) CircleCon(n,MidPoint,Radius);
end

%% Optimization and Prediction

for i = 1
    % negative log marginal likelihood as objective function
    if Mode == "GP"
        tic;
        LogLikelihood = @(p) getLogLikelihood(Xa,Ya(i,:),Kernel,p(1),p(2),p(3));
        s = fmincon(LogLikelihood,[1,1,1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
        time = toc;
        
        NoiseStd(i) = s(1);
        s0(i) = s(2);
        s1(i) = s(3);
    elseif Mode == "SPGP"
        tic;
        PreLogLikelihood = @(t) getLogLikelihood(Xa,Ya(i,:),Kernel,t(1),t(2),t(3));
        u = fmincon(PreLogLikelihood,[1,1,1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
        
        LogLikelihood = @(p) getSparseLogLikelihood(Xa,Ya(i,:),Kernel,[p;ones(1,m)],u(1),u(2),u(3));
        s = fmincon(LogLikelihood,Si,[],[],[],[],[],[],nonlcon,options);
        time = toc;
        
        NoiseStd(i) = u(1);
        s0(i) = u(2);
        s1(i) = u(3);
        Xi(:,:,i) = [s;ones(1,m)];
    end
        
    if ShowResults == true        
        % prediction at testing data
        if Mode == "GP"
            [Mean,Covariance,LogLikelihood] = GaussianProcess(Xa,Ya(i,:),Xt,Kernel, ...
                NoiseStd(i),s0(i),s1(i));
        elseif Mode == "SPGP"
            [Mean,Covariance,LogLikelihood] = SparseGaussianProcess(Xa,Ya(i,:),Xt,Kernel, ...
                Xi(:,:,i),NoiseStd(i),s0(i),s1(i));
        end
        
        figure();
        
        subplot(1,3,1);
        title("Flight Offset");
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Range Offset [m]");
        xlim([-1.5,1.5]);
        ylim([-1.5,1.5]);
        hold on;
        
        % offset
        scatter3(Xa(1,:),Xa(2,:),Ya(i,:),5,'k+')

        grid on;
        hold off;
        
        subplot(1,3,2);
        title("Flight Space Offset Evaluation");
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Range Offset Prediction [m]");
        hold on;
        
        % offset evaluation
        scatter3(Xt(1,:),Xt(2,:),Mean,10,Mean)
        colormap(gca,'gray');
        
        grid on;
        hold off;
        
        subplot(1,3,3);
        title("Flight Space Covariance Evaluation");
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Standard Deviation Prediction [cm]");
        hold on;
        
        % covariance evaluation
        Std = sqrt(diag(Covariance))*100;
        scatter3(Xt(1,:),Xt(2,:),Std,10,Std);
        colormap(gca,'jet');
        
        grid on;
        hold off;
        
        % results
        if Mode == "GP"
            disp("Kernel: RBFKernel");
            disp("Training time: "+time+" seconds");
            disp("Final negative log marginal likelihood: "+LogLikelihood);
            disp("Number of training points: "+size(Xa,2));
            disp("Estimated noise standard deviation: "+NoiseStd(i));
            disp("Kernel hyperparameters: "+s0(i)+"/"+s1(i));
        elseif Mode == "SPGP"
            disp("Kernel: RBFKernel");
            disp("Training time: "+time+" seconds");
            disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
            disp("Number of training points: "+size(Xa,2));
            disp("Estimated noise standard deviation: "+NoiseStd(i));
            disp("Kernel hyperparameters: "+s0(i)+"/"+s1(i));
        end
    end
end

if Save == true
    AnchorPos = AnchorPos';
    if Mode == "GP"
        save('HyperparametersGP.mat','Xd','Xa','Yd','Ya','AnchorPos','NoiseStd','s0','s1');
    elseif Mode == "SPGP"
        save('HyperparametersSPGP.mat','Xd','Xa','Yd','Ya','AnchorPos','NoiseStd','s0','s1','Xi');
    end
end