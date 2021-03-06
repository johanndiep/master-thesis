% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with UWB measurement feedback. Thereby,
% the (Sparse) Gaussian Process measurement model is used. In order to 
% get the trained hyperparameters for each (Sparse) Gaussian Process model, run
% "CovEvalMain.m" first.

classdef ConstantVelocityGP < handle
    properties
        Xd
        Yd
        Kernel
        Xi
        NoiseVar
        s0
        s1
        AnchorPos
        ParamGP
        ParamSPGP
        Q
        X
        P
    end
    
    methods
        % Initialize the EKF with the corresponding parameters and matrices 
        % for the process and measurement model.
        %   - Xd: Training data in form (3 x n)
        %   - Yd: Response training data in form (6 x n)
        %   - Kernel: Corresponding kernel function handle
        %   - NoiseStd: Noise standard deviation in form (1 x 6)
        %   - s0/s1: Scalar kernel parameters in form (1 x 6)
        %   - AnchorPos: The position of the 6 anchors in form (6 x 3)
        %   - Xi: Pseudo-input data in form (3 x m x 6)
        function Model = ConstantVelocityGP(Xd,Yd,Kernel,NoiseStd,s0,s1,AnchorPos,RangeArray,Xi)
            if nargin == 8
                Xi = ones(3,1,6);
            end
            
            Model.Xd = Xd;
            Model.Yd = Yd;
            Model.Kernel = Kernel;
            Model.Xi = Xi;
            Model.NoiseVar = NoiseStd.^2;
            Model.s0 = s0;
            Model.s1 = s1;
            Model.AnchorPos = AnchorPos;
            
            if nargin == 8
                Model.ParamGP = Model.ParameterGP;
            else
                Model.ParamSPGP = Model.ParameterSPGP;
            end
          
            ObjNorm = @(p) getTriangulationNorm(RangeArray,AnchorPos,p);
            p = fmincon(ObjNorm,[0;0;0],[],[],[],[],[],[],[]);            
            
            Model.X = [p(1);0;p(2);0;p(3);0]; % [px;vx;py;vy;pz;vz]
            Model.P = 0.1*eye(6);
        end

        % In total, there are 6 different Gaussian Process models, one
        % for each anchor. Xd is a matrix which correspond to the 
        % ground-truth positions of the tag measured by the VICON 
        % capture system and Yd is the oberservation matrix which stores 
        % the offset range measurements between the tag and the 
        % corresponding anchor. Thereby, this method calculates all the 
        % necessary model parameters for the Gaussian Process using the 
        % distance kernel.
        %   - Model: Model object defined by the constructor
        function ParamGP = ParameterGP(Model)
            Xd = Model.Xd;
            Yd = Model.Yd;
            Kernel = Model.Kernel;
            NoiseVar = Model.NoiseVar;
            s0 = Model.s0;
            s1 = Model.s1;
            AnchorPos = Model.AnchorPos;
            
            for i = 1:6
                ParamGP(i) = GaussianModel(Xd,Yd(i,:),Kernel, ...
                    NoiseVar(i),s0(i),s1(i));
            end
        end
        
        % This method is similiar to the "ParameterGP"-method, except that
        % it calculates all the necessary model parameters for the Sparse
        % Gaussian Process using the distance kernel.
        %    - Model: Model object defined by the constructor
        function ParamSPGP = ParameterSPGP(Model)
            Xd = Model.Xd;
            Yd = Model.Yd;
            Kernel = Model.Kernel;
            Xi = Model.Xi;
            NoiseVar = Model.NoiseVar;
            s0 = Model.s0;
            s1 = Model.s1;
            AnchorPos = Model.AnchorPos;
            
            for i = 1:6
                ParamSPGP(i) = SparseGaussianModel(AnchorPos(i,:)'-Xd,Yd(i,:), ...
                    Kernel,AnchorPos(i,:)'-Xi(:,:,i),NoiseVar(i),s0(i),s1(i));
            end
        end
        
        % For a given position, this method outputs its UWB range offset 
        % distributation by a mean vector and covariance matrix according 
        % to Gaussian Process using the distance kernel. Besides, it also 
        % returns the conventional measurement prediction for analysis purpose.
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function [h,R,Abs,CovVal] = PredictionGP(Model,Xp)
            Kernel = Model.Kernel;
            AnchorPos = Model.AnchorPos;
            ParamGP = Model.ParamGP;

            Px = Xp(1:2:end);
            
            Diff = repmat(Px,1,6)-AnchorPos';
            Abs = vecnorm(Diff)';
            
            Mean = zeros(6,1);
            CovVal = zeros(6,1);
            for i = 1:6
                [Mean(i),CovVal(i),~] = GaussianPrediction(ParamGP(i),Px,Kernel);
            end
            
            h = Abs-Mean;
            R = diag(CovVal);
        end
        
        % For a given position, this method outputs its UWB range offset 
        % distribution by a mean vector and covariance matrix according to
        % Sparse Gaussian Process using the distance kernel. Besides, it 
        % also returns the conventional measurement prediction for analysis 
        % purpose.
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function [h,R,Abs,CovVal] = PredictionSPGP(Model,Xp)
            Kernel = Model.Kernel;
            AnchorPos = Model.AnchorPos;
            ParamSPGP = Model.ParamSPGP;
            
            Px = Xp(1:2:end);
            
            Diff = repmat(Px,1,6)-AnchorPos';
            Abs = vecnorm(Diff)';
            
            Mean = zeros(6,1);
            CovVal = zeros(6,1);
            for i = 1:6
                [Mean(i),CovVal(i),~] = SparseGaussianPrediction(ParamSPGP(i), ...
                    AnchorPos(i,:)'-Px,Kernel);
            end
            
            h = Abs-Mean;
            R = diag(CovVal);
        end
        
        % Error corrected measurement model and its linearization about
        % current state with Gaussian Process. Hereby, the distance Kernel is
        % used.
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function H = ErrorCorrectionGP(Model,Xp)
            Kernel = Model.Kernel;
            AnchorPos = Model.AnchorPos;            
            ParamGP = Model.ParamGP;
            Xd = Model.Xd;
            s0 = Model.s0;
            s1 = Model.s1;
            
            Px = Xp(1:2:end);
            Hs = zeros(6,6);
            Hg = zeros(6,6);
            
            Num = repmat(Px,1,6)-AnchorPos';
            Denom = vecnorm(Num);
            V = (Num./Denom)';
            Hs(:,1) = V(:,1); Hs(:,3) = V(:,2); Hs(:,5) = V(:,3);
            
            for i = 1:6
                K = Kernel(Px,Xd,s0(i),s1(i));
                
                A = -1/s1(i)*(Px-Xd);                
                
                E = repmat(K,3,1).*A;
                
                Kderiv = zeros(size(Xd,2),6);
                Kderiv(:,1) = E(1,:)'; 
                Kderiv(:,3) = E(2,:)'; 
                Kderiv(:,5) = E(3,:)';

                
                a = ParamGP(i).a;
                Hg(i,:) = a'*Kderiv;
            end
            
            H = Hs-Hg;
        end      
        
        % Error corrected measurement model and its linearization about
        % current state with Sparse Gaussian Process. Hereby, the distance
        % kernel is used.
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)        
        function H = ErrorCorrectionSPGP(Model,Xp)
            Kernel = Model.Kernel;
            AnchorPos = Model.AnchorPos;            
            ParamSPGP = Model.ParamSPGP;
            Xi = Model.Xi;
            s0 = Model.s0;
            s1 = Model.s1;
            
            Px = Xp(1:2:end);
            Hs = zeros(6,6);
            Hg = zeros(6,6);            
            
            Num = repmat(Px,1,6)-AnchorPos';
            Denom = vecnorm(Num);
            V = (Num./Denom)';
            Hs(:,1) = V(:,1); Hs(:,3) = V(:,2); Hs(:,5) = V(:,3);
            
            for i = 1:6
                K = Kernel(AnchorPos(i,:)'-Px,AnchorPos(i,:)'-Xi(:,:,i),s0(i),s1(i));

                n1 = vecnorm(AnchorPos(i,:)'-Px);
                n2 = vecnorm(AnchorPos(i,:)'-Xi(:,:,i));
                
                A = 1/s1(i)*(n1-n2)*1/n1.*(AnchorPos(i,:)'-Px);
                
                E = repmat(K,3,1).*A;
                
                Kderiv = zeros(size(Xi,2),6);
                Kderiv(:,1) = E(1,:)'; 
                Kderiv(:,3) = E(2,:)'; 
                Kderiv(:,5) = E(3,:)';
                
                C = ParamSPGP(i).C;
                Hg(i,:) = C'*Kderiv;
            end

            H = Hs-Hg;
        end
        
        % Executes the EKF prior update step.
        %   - Model: Model object defined by the constructor
        %   - dT: Time interval between each EKF iteration        
        function UpdatePrior(Model,dT)
            X = Model.X;
            P = Model.P;

            SigmaNoise = 2;
            q = [0.25*dT^4,0.5*dT^3;0.5*dT^3,dT^2]*SigmaNoise;
            Q = blkdiag(q,q,q);
            
            a = [1,dT;0,1];
            A = blkdiag(a,a,a);
            
            Model.X = A*X;
            Model.P = A*P*A'+Q;
        end
        
        % Executes the EKF posterior update step.
        %   - Model: Model object defined by the constructor
        %   - Z: Range measurements from UWB in form (6 x 1)
        %   - Mode: Choice between regular GP or SPGP
        function [CurPos,CurVel,t,Abs,CovVal] = UpdateMeasurement(Model,Z,Mode)
           X = Model.X;
           P = Model.P;
           
           if Mode == "GP"
               [h,R,Abs,CovVal] = Model.PredictionGP(X);
               H = Model.ErrorCorrectionGP(X);
           elseif Mode == "SPGP"
               [h,R,Abs,CovVal] = Model.PredictionSPGP(X);
               H = Model.ErrorCorrectionSPGP(X);                
           end
           
           t = h;
                      
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