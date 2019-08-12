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
        %   - Xi: State initialization e.g. with Gauss-Newton algorithm
        function Model = ConstantVelocityEKF(dT,Xi)
            a = [1,dT;0,1];
            VarianceAcc = 0.125;
            q = [VarianceAcc*dT^4/3,VarianceAcc*dT^3/2;VarianceAcc*dT^3/2,dT/2];
            
            Model.A = blkdiag(a,a,a);
            Model.H = [1,0,0,0,0,0;0,0,1,0,0,0;0,0,0,0,1,0];
            Model.Q = blkdiag(q,q,q);
            Model.R = diag([0.01,0.01,0.01]);

            Model.X = Xi;
            Model.P = 10*eye(6);
        end
        
        % EKF prior update equations 
        %   - Model: Model object defined by the constructor
        function UpdatePrior(Model)  
            Model.X = Model.A*Model.X;
            Model.P = Model.A*Model.P*Model.A'+Model.Q;
        end
        
        % EKF measurement update equations
        %   - Model: Model object defined by the constructor
        %   - Z: Measurement from VICON system
        function State = UpdateMeasurement(Model,Z)
            S = Model.H*Model.P*Model.H'+Model.R;
            K = Model.P*Model.H'/S;
            
            Model.X = Model.X+K*(Z-Model.H*Model.X);
            Model.P = (eye(6)-K*Model.H)*Model.P;
            
            State = Model.X;
        end
    end
end