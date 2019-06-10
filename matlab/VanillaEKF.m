% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This is an implementation of an extended Kalman Filter for nonlinear
% dynamics systems. The function returns the state estimate x and
% covariance P for a constant velocity model:
%
%   - x_k = q(x_{k-1},v_{k-1}), v_{k-1} ~ N(0,Q)
%   - z_k = h(x_{k},w_{k}), w ~ N(0,R)
%
% Inputs:
%   - x_posterior: a priori state estimate, posterior estimate from previous iteration
%   - P_posterior: a priori estimated state covariance, posterior estimate from previous iteration
%   - z: current measurement
%   - dT: time interval between measurements
%
% Outputs:
%   - x_posterior: a posteriori state estimate
%   - P_posterior: a posteriori state covariance

function [x_posterior,P_posterior] = VanillaEKF(anchor_pos,x_posterior,P_posterior,dT,z)
%% Symbolic values definition

    % position and velocity parameters
    syms p_x p_y p_z
    syms v_x v_y v_z
    x = [p_x,p_y,p_z,v_x,v_y,v_z];

%% Process and measurement model

    % system matrix for constant velocity model
    A = [1,0,0,dT,0,0; ...
        0,1,0,0,dT,0; ...
        0,0,1,0,0,dT; ...
        0,0,0,1,0,0; ...
        0,0,0,0,1,0; ...
        0,0,0,0,0,1];
    
    % non-linear measurement prediction model
    index = 1; 
    r = sym(zeros(size(anchor_pos,1),1));
    for i = 1:size(anchor_pos,1)
        r(index) = sqrt((x(1)-anchor_pos(i,1))^2+(x(3)-anchor_pos(i,2))^2+(x(5)-anchor_pos(i,3))^2);
        index = index + 1;
    end
    
    % measurement matrix
    H = jacobian(r,x);
    H = matlabFunction(H);
    H = convertToAcceptArray(H);
    
    Q = diag([0,0,0,1,1,1]); % process noise covariance
    R = eye(size(anchor_pos,1))*0.1; % measurement noise covariance
    
    
%% Prior update

    x_prior = A*x_posterior; % project the state ahead
    P_prior = A*P_posterior*A'+Q; % project the error covariance ahead
    
%% A posteriori update
    
    K = P_prior*H(x_prior(1:3)')'*(H(x_prior(1:3)')*P_prior*H(x_prior(1:3)')'+R)^(-1); % compute the Kalman gain
    
    x_posterior = x_prior+K*(z-H(x_prior(1:3)')*x_prior); % update the estimate with measurement z
    P_posterior = (eye(6)-K*H(x_prior(1:3)'))*P_prior; % update the error covariance
end
   