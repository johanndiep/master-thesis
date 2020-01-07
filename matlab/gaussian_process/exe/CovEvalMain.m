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
%   - The true ranging distance is an approximation, since the markers are
%     not planarly placed on the antennas. Does this influence the
%     prediction in any way? Can we also achieve good accuracy with an 
%     estimate of anchor positions?
%     [Yes, GP can calibrate those systematic errors away.]
%   - Influence of kernel functions.
%     [Depending on which kernel function one chooses, we get more or less
%     non-linear behaviour around the space where we have data
%     measurements.
%   - SPGP does not converge sometimes, why? 

clear; clc;

% load('UWBCircConDataGP.mat'); % UWB and VICON measurements
% load('UWBYawCircConDataGP.mat'); % UWB and VICON measuremements with yaw
% load('UWBCenCircConDataGP.mat'); % UWB and VICON centered measurements
% load('UWBSplineConDataGP.mat'); % UWB and VICON measurements at spline
load('UWBPosConDataGP.mat'); % UWB and Vicon measurements for position holding

%% Data Preprocessing

% removing zero measurements
[r,c] = find(SaveRangeArr==0);
SaveRangeArr(:,c) = [];
SaveViconPos(:,c) = [];
SaveViconQuat(:,c) = [];

% data mapping
DataPrepObj = DataPrep(SaveRangeArr);
[X,Y,P] = DataPrepObj.SimplifiedFlight(SaveViconPos,AnchorPos);

Downsample = 1;
X = X(:,1:Downsample:end);
Y = Y(:,1:Downsample:end);

Kernel = @RBFKernel;
% Kernel = @DistanceKernel;

% pre-allocation
NoiseStd = zeros(1,6);
s0 = zeros(1,6);
s1 = zeros(1,6);

% testing space and testing data
TestingPoints = 30;
a = linspace(MidPoint(1)-Radius-1,MidPoint(1)+Radius+1,TestingPoints);
b = linspace(MidPoint(2)-Radius-1,MidPoint(2)+Radius+1,TestingPoints);
[x,y] = meshgrid(a,b);
Xt(1,:) = x(:)';
Xt(2,:) = y(:)';
Xt(3,:) = ones(size(Xt(1,:)));

ShowResults = true;
Save = false;
Mode = "GP";
options = optimoptions('fmincon','Display','iter','Algorithm','interior-point');

% generate pseudo-inputs
if Mode == "SPGP"
    m = 20;
    
    c = linspace(0,2*pi,100);
    x = MidPoint(1)+Radius*sin(c);
    y = MidPoint(2)-Radius*cos(c);
    
    RandIndex = randperm(length(c),m);
    Si = [x(1,RandIndex);y(1,RandIndex)];
    
    nonlcon = @(n) CircleCon(n,MidPoint,Radius);
    
    Xi = zeros(3,m,6);
end

%% Optimization and Prediction

for i = 1:6
    % negative log marginal likelihood as objective function
    if Mode == "GP"
        tic;
        LogLikelihood = @(p) getLogLikelihood(X,Y(i,:),Kernel,p(1),p(2),p(3));
        s = fmincon(LogLikelihood,[1,1,1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
        time = toc;
        
        NoiseStd(i) = s(1);
        s0(i) = s(2);
        s1(i) = s(3);
    elseif Mode == "SPGP"
        tic;
        PreLogLikelihood = @(t) getLogLikelihood(AnchorPos(i,:)'-X,Y(i,:),Kernel,t(1),t(2),t(3));
        u = fmincon(PreLogLikelihood,[1,1,1],[],[],[],[],[0,0,0],[Inf,Inf,Inf],[],options);
        
        LogLikelihood = @(p) getSparseLogLikelihood(AnchorPos(i,:)'-X,Y(i,:),Kernel, ...
            AnchorPos(i,:)'-[p;ones(1,m)],u(1),u(2),u(3));
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
            [Mean,Covariance,LogLikelihood] = GaussianProcess(X,Y(i,:), ...
                Xt,Kernel,NoiseStd(i),s0(i),s1(i));
        elseif Mode == "SPGP"
            [Mean,Covariance,LogLikelihood] = SparseGaussianProcess(AnchorPos(i,:)'-X,Y(i,:), ...
                AnchorPos(i,:)'-Xt,Kernel,AnchorPos(i,:)'-Xi(:,:,i),NoiseStd(i),s0(i),s1(i));
        end
        
        figure();
        
%         subplot(1,3,1);
%         title("Flight Offset");
%         xlabel("x-Axis [m]");
%         ylabel("y-Axis [m]");
%         zlabel("Range Offset [m]");
%         xlim([MidPoint(1)-Radius-1,MidPoint(1)+Radius+1]);
%         ylim([MidPoint(2)-Radius-1,MidPoint(2)+Radius+1]);
%         hold on;
%         
%         % offset
%         scatter3(X(1,:),X(2,:),Y(i,:),5,'k+');
% 
%         view(35.1654,48.1915)
%         grid on;
%         hold off;
        
        subplot(1,2,1);
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Mean [m]");
        hold on;
        
        % offset evaluation
        surf(x,y,vec2mat(Mean,TestingPoints));
        
        view(35.1654,48.1915)
        grid on;
        hold off;
        
        subplot(1,2,2);
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("Standard Deviation [m]");
        hold on;
        
        % covariance evaluation
        Cov = diag(sqrt(Covariance));
        surf(x,y,vec2mat(Cov,TestingPoints));
        
        view(35.1654,48.1915)
        grid on;
        hold off;
        
        % results
        if Mode == "GP"
            disp("Kernel: RBFKernel");
            disp("Training time: "+time+" seconds");
            disp("Final negative log marginal likelihood: "+LogLikelihood);
            disp("Number of training points: "+size(X,2));
            disp("Estimated noise standard deviation: "+NoiseStd(i));
            disp("Kernel hyperparameters: "+s0(i)+"/"+s1(i));
        elseif Mode == "SPGP"
            disp("Kernel: DistanceKernel");
            disp("Training time: "+time+" seconds");
            disp("Final negative sparse log marginal likelihood: "+LogLikelihood);
            disp("Number of training points: "+size(X,2));
            disp("Estimated noise standard deviation: "+NoiseStd(i));
            disp("Kernel hyperparameters: "+s0(i)+"/"+s1(i));
        end
    end
end 

save('PositionHypGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1');

if Save == true
    if Mode == "GP"
        if ChangeHeading == false
            save('CircleHypGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1');
        else
            if PointToCenter == false
                if SplineFlight == false
                    save('YawCircleHypGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1');
                else
                    save('SplineHypGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1');
                end
            else
                save('CenCircleHypGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1');
            end
        end
    elseif Mode == "SPGP"
        if ChangeHeading == false
            save('CircleHypSPGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1','Xi');
        else
            if PointToCenter == false
                if SplineFlight == false
                    save('YawCircleHypSPGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1','Xi');
                else
                    save('SplineHypSPGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1','Xi');
                end
            else
                save('CenCircleHypSPGP.mat','X','Y','AnchorPos','NoiseStd','s0','s1','Xi');
            end
        end
    end
end