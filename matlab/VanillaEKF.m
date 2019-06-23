% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This is an implementation of an extended Kalman Filter for nonlinear
% dynamics systems. The function returns the state estimate x and
% covariance P for a constant velocity model.
%
%   - x(k) = q[x(k-1),v(k-1)], v(k-1) ~ N(0,Q)
%   - z(k) = h[x(k),w(k)], w(k) ~ N(0,R)
%
% Inputs:
%   - NumberOfAnchors: Amount of anchors in the setup
%   - x_Posterior: A priori state estimate, posterior estimate from previous iteration
%   - P_Posterior: A priori estimated state covariance, posterior estimate from previous iteration
%   - z: current measurement
%   - DeltaT: time interval between measurements
%   - h: non-linear measurement prediction model
%   - H: Jacobian of non-linear measurement prediction model
%
% Outputs:
%   - x_Posterior: A posteriori state estimate
%   - P_Posterior: A posteriori state covariance

function [x_Posterior,P_Posterior] = VanillaEKF(NumberOfAnchors,x_Posterior,P_Posterior,DeltaT,z,h,H)  
    % system matrix for constant velocity model
    A = [1,0,0,DeltaT,0,0;0,1,0,0,DeltaT,0;0,0,1,0,0,DeltaT;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1];
    
    % process noise covariance
    Q_1= 0.125 * DeltaT^3/3;
    Q_2 = 0.125 * DeltaT^2/2;
    Q_3 = DeltaT;
    Q = [Q_1,0,0,Q_2,0,0;0,Q_1,0,0,Q_2,0;0,0,Q_1,0,0,Q_2;Q_2,0,0,Q_3,0,0;0,Q_2,0,0,Q_3,0;0,0,Q_2,0,0,Q_3];

    % measurement noise covariance, increasing variance for zero measurements
    R = eye(NumberOfAnchors)*500;
    if ~all(z)
       ZeroRows = find(z==0);
       DiagonalElements = diag(R);
       DiagonalElements(ZeroRows) = 10e10;
       R = diag(DiagonalElements);
    end
        
    x_Prior = A*x_Posterior; % project the state ahead
    P_Prior = A*P_Posterior*A'+Q; % project the error covariance ahead
            
    K = P_Prior*H(x_Prior(1:3)')'*(H(x_Prior(1:3)')*P_Prior*H(x_Prior(1:3)')'+R)^(-1); % compute the Kalman gain
        
    x_Posterior = x_Prior+K*(z-h(x_Prior(1:3)')); % update the estimate with measurement z
    P_Posterior = (eye(NumberOfAnchors)-K*H(x_Prior(1:3)'))*P_Prior; % update the error covariance
end
   