% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with VICON measurement feedback.

classdef ConstantVelocityEKF < handle
    properties
        H
        R
        X
        P
    end
    
    methods
        % Initialize the EKF with the corresponding matrices for the
        % process and measurement model.
        function Model = ConstantVelocityEKF()
            Model.H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
            
            Model.R = diag([0.01,0.01,0.01]);

            Model.X = zeros(6,1);  % [px;vx;py;vy;pz;vz]
            Model.P = 10*eye(6);
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
        %   - Z: Measurement from VICON system in form (3 x 1)
        function [CurPos,CurVel] = UpdateMeasurement(Model,Z)
            X = Model.X;            
            P = Model.P;
            H = Model.H;            
            R = Model.R;
            
            S = H*P*H'+R;
            K = P*H'/S;
            
            Model.X = X+K*(Z-H*X);
            Model.P = (eye(6)-K*H)*P;
            
            Xp = Model.X;
            CurPos = Xp(1:2:end);
            CurVel = Xp(2:2:end);
        end
    end
end