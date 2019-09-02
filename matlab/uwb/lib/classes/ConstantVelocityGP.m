% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with UWB measurement feedback. Thereby,
% the Gaussian Process measurement model is used.

classdef ConstantVelocityGP < handle
    properties
        Xd
        Yd
        Kernel
        Xid
        NoiseVar
        s0
        s1
        s2
        ParamSPGP
        Q
        X
        P
    end
    
    methods
        % Initialize the EKF with the corresponding matrices for the 
        % process and measurement model.
        %   - Xd: Training data in form (3 x n)
        %   - Yd: Response training data in form (6 x n)
        %   - Kernel: Corresponding kernel function handle
        %   - Xid: Pseudo-input data in form (3 x m x 6)
        %   - NoiseStd: Noise standard deviation in form (1 x 6)
        %   - s0/s1/s2: Scalar kernel parameters in form (1 x 6)
        function Model = ConstantVelocityGP(Xd,Yd,Kernel,Xid,NoiseStd,s0,s1,s2)
            if nargin == 7
                s2 = ones(1,6);
            end
            
            Model.Xd = Xd;
            Model.Yd = Yd;
            Model.Kernel = Kernel;
            Model.Xid = Xid;
            Model.NoiseVar = NoiseStd^2;
            Model.s0 = s0;
            Model.s1 = s1;
            Model.s2 = s2;
            
            Model.ParamSPGP = Model.ParameterSPGP;

            q = [0,0;0,1];
            Model.Q = blkdiag(q,q,q);
            
            Model.X = zeros(6,1); % (px,vx,py,vy,pz,vz)
            Model.P = 10*eye(6);
        end

        % In total, there are 6 different Gaussian Process models, one
        % for each anchor. Xd is a matrix which correspond to the 
        % ground-truth positions measured by the VICON capture system 
        % and Yd is the oberservation matrix which stores the range
        % measurements between the tag and the corresponding anchor. 
        % Thereby, this method calculate all the necessary model 
        % parameters for Sparse Gaussian Process regressions.
        %   - Model: Model object defined by the constructor
        function ParamSPGP = ParameterSPGP(Model)
            Xd = Model.Xd;
            Yd = Model.Yd;
            Kernel = Model.Kernel;
            Xid = Model.Xid;
            NoiseVar = Model.NoiseVar;
            s0 = Model.s0;
            s1 = Model.s1;
            s2 = Model.s2;            
            
            for i = 1:6
                ParamSPGP(i) = SparseGaussianModel(Xd,Yd(i,:),Kernel,Xi(:,:,i), ...
                    NoiseVar(i),s0(i),s1(i),s2(i));
            end
        end
        
        % For a given position, this method outputs its UWB measurement 
        % distributation by a mean vector and covariance matrix.
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function [Mean,Covariance] = PredictionSPGP(Model,Xp)
            ParamSPGP = Model.ParamSPGP;
            Kernel = Model.Kernel;
            
            Px = Xp(1:2:end);
            
            Mean = zeros(6,1);
            CovVal = zeros(6,1);
            
            for i = 1:6
                [Mean(i),CovVal(i),~] = SparseGaussianPrediction(ParamSPGP(i),Px,Kernel);
            end
            
            Covariance = diag(CovVal);
        end
        
        % Error corrected measurement model and its linearization about
        % current state with Gaussian Process
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function H = ErrCorrMeasModel(Model,Xp)
            Xid = Model.Xid;
            s0 = Model.s0;
            s1 = Model.s1;
            s2 = Model.s2;
            ParamSPGP = Model.ParamSPGP;
            
            Px = Xp(1:2:end);
            
            H = zeros(6,6);
            
            for i = 1:6
                C = ParamSPGP(i).C;
                KernDer = PosePerKernDeriv(Px,Xid(:,:,i),s0,s1,s2);
                H(i,:) = C'*KernDer;
            end
        end      
        
        % Executes the EKF prior update step.
        %   - Model: Model object defined by the constructor
        %   - dT: Time interval between each EKF iteration        
        function UpdatePrior(Model,dT)
            X = Model.X;
            P = Model.P;
            Q = Model.Q;
            
            a = [1,dT;0,1];
            A = blkdiag(a,a,a);
            
            Model.X = A*X;
            Model.P = A*P*A'+Q;
        end
        
        % Executes the EKF posterior update step.
        %   - Model: Model object defined by the constructor
        %   - Z: Range measurements from UWB in form (6 x 1)        
        function [CurPos,CurVel] = UpdateMeasurement(Model,Z)
           X = Model.X;
           P = Model.P;
           
           [h,R] = Model.PredictionSPGP(X);
           H = Model.ErrCorrMeasModel(X);
           
           ZeroInd = find(Z==0);
           if isempty(ZeroInd) == 0
               Z(ZeroInd) = [];
               
               R(ZeroInd,:) = []; R(:,ZeroInd) = [];
               
               h(ZeroInd) = [];
               H(ZeroInd,:) = [];
           end
           
           S = H*P*H'+R;
           K = P*H'/S;
           
           Model.X = X+K*(Z-h);
           Model.P = (eye(6)-K*H)*P;
           
           Xp = Model.X;
           CurPos = Xp(1:2:end);
           CurVel = Xp(2:2:end);
        end
    end
end