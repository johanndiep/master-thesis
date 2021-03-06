% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with UWB measurement feedback. Thereby,
% the conventional measurement model is used.

classdef ConstantVelocityUWB < handle
    properties
        AnchorPos
        R
        X
        P
    end
    
    methods
        % Initialize the EKF with the anchor positions and the corresponding 
        % matrices for the process and measurement model.
        %   - AnchorPos: The position of the 6 anchors in form (6 x 3)
        %   - RangeArray: Initial range measurements from UWB in form (6 x 1)
        function Model = ConstantVelocityUWB(AnchorPos,RangeArray)
            Model.AnchorPos = AnchorPos;
            
            Model.R = diag([0.05,0.05,0.05,0.05,0.05,0.05]);
            
            ObjNorm = @(p) getTriangulationNorm(RangeArray,AnchorPos,p);
            p = fmincon(ObjNorm,[0;0;0],[],[],[],[],[],[],[]);
            
            Model.X = [p(1);0;p(2);0;p(3);0]; % [px;vx;py;vy;pz;vz]
            Model.P = 0.1*eye(6);
        end
        
        % Outputs the measurement model and its linearization 
        % about current state.
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
        function [CurPos,CurVel] = UpdateMeasurement(Model,Z)
           X = Model.X;
           P = Model.P;
           R = Model.R;
           
           [h,H] = Model.LinMeasurementModel(X);
           
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