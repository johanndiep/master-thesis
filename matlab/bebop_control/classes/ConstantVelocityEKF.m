% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with VICON measurement feedback.

classdef ConstantVelocityEKF < handle
    properties
        H
        Q
        R
        X
        P
    end
    
    methods
        
        % Initialize the EKF with the corresponding matrices for the
        % process and measurement model
        function Model = ConstantVelocityEKF()
            q = [0,0;0,1];
            
            Model.H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
            
            Model.Q = blkdiag(q,q,q);
            Model.R = diag([0.01,0.01,0.01]);

            Model.X = zeros(6,1); % (px,vx,py,vy,pz,vz)
            Model.P = 10*eye(6);
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
        
        % EKF measurement update equations
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