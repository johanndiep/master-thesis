% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with UWB measurement feedback.

classdef ConstantVelocityUWB < handle
    properties
        AnchorPos
        Q
        R
        X
        P
        ParamSPGP
    end
    
    methods
        % Initialize the EKF with the corresponding matrices for the 
        % process and measurement model.
        %   - AnchorPos: The position of the 6 anchors in form (6 x 3)
        function Model = ConstantVelocityUWB(AnchorPos)
            Model.AnchorPos = AnchorPos;
            
            q = [0,0;0,1];
            Model.Q = blkdiag(q,q,q);
            Model.R = diag([0.05,0.05,0.05,0.05,0.05,0.05]);
            
            Model.X = zeros(6,1); % (px,vx,py,vy,pz,vz)
            Model.P = 10*eye(6);
        end
        
        % Measurement model and its linearization about current state
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function [h,H] = LinMeasurementModel(Model,Xp)  
            AnchorPos = Model.AnchorPos;
            
            Px = Xp(1:2:end);
            Num = repmat(Px,1,6)-AnchorPos';
            Denom = vecnorm(Num);
            
            V = (Num./Denom)';
            Z = zeros(6,1);
            
            h = Denom';
            H = [V(:,1),Z,V(:,2),Z,V(:,3),Z];   
        end

        % In total, there are 6 different Gaussian Process models, one
        % for each anchor. This method returns all the necessary model
        % parameter for Sparse Gaussian Process regression for each of 
        % these anchor data.
        %   - Model: Model object defined by the constructor
        %   - Xd: Training data in form (d x n x 6)
        %   - Yd: Response training data in form (1 x n x 6)
        %   - Kernel: Corresponding kernel function handle
        %   - Xid: Pseudo-input data in form (d x n x 6)
        %   - NoiseVar: Noise variance in form (1 x 6)
        %   - s0/s1/s2: Scalar kernel parameters in form (1 x 6)
        function ParameterSPGP(Model,Xd,Yd,Kernel,Xi,NoiseVar,s0,s1,s2)
            for i = 1:6
                ParamSPGP(i) = SparseGaussianModel(Xd(:,:,i),Yd(:,:,i), ...
                    Kernel,Xi(:,:,i),NoiseVar(i),s0(i),s1(i),s2(i));
            end
            
            Model.ParamSPGP = ParamSPGP;
        end
        
        % Error corrected measurement model and its linearization about
        % current state with Gaussian Process
        %   - Model: Model object defined by the constructor
        %   - Xp: Prior state estimate in form (6 x 1)
        function [h,H] = ErrCorrMeasModel(Model,Xp)
            
            
            
            h = [];
            H = [];
        end
        
        % EKF prior update equations 
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
        
        function [CurPos,CurVel] = UpdateMeasurement(Model,Z)
           X = Model.X;
           P = Model.P;
           R = Model.R;
           
           [h,H] = Model.LinMeasurementModel(X);
           
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