% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% Constant velocity model EKF with VICON measurement feedback.

classdef ConstantVelocityEKF < handle
    properties
        A
        H
        Q
        R
        X
        P
    end
    
    methods
        
        % Initialize the EKF with the corresponding matrices for the
        % process and measurement model
        %   - dT: Time interval between each EKF iteration
        function Model = ConstantVelocityEKF(dT)
            a = [1,dT;0,1];
            q = [0,0;0,1];
            
            Model.A = blkdiag(a,a,a);
            Model.H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
            
            Model.Q = blkdiag(q,q,q);
            Model.R = diag([0.01,0.01,0.01]);

            Model.X = zeros(6,1);
            Model.P = 10*eye(6);
        end
        
        % EKF prior update equations 
        %   - Model: Model object defined by the constructor
        function UpdatePrior(Model)
            A = Model.A;
            X = Model.X;
            P = Model.P;
            Q = Model.Q;
            
            Model.X = A*X;
            Model.P = A*P*A'+Q;
        end
        
        % EKF measurement update equations
        %   - Model: Model object defined by the constructor
        %   - Z: Measurement from VICON system in form (3 x 1)
        function [Pos,Vel] = UpdateMeasurement(Model,Z)
            H = Model.H;
            P = Model.P;
            R = Model.R;
            X = Model.X;
            
            S = H*P*H'+R;
            K = P*H'/S;
            
            Model.X = X+K*(Z-H*X);
            Model.P = (eye(6)-K*H)*P;
            
            Xp = Model.X;
            Pos = Xp(1:2:end);
            Vel = Xp(2:2:end);
        end
    end
end