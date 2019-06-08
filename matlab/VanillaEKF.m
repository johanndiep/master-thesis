% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This is an implementation of an extended Kalman Filter for nonlinear
% dynamics systems. The function returns the state estimate x and
% covariance P for nonlinear dynamics system of form:
%   - x_k = q(x_{k-1},v_{k-1}), v_{k-1} ~ N(0,Q)
%   - z_k = h(x_{k},w_{k}), w ~ N(0,R)
% Here, a constant velocity model is implemented.
% Inputs:
%   - x_prior: a priori state estimate
%   - P_prior: a priori estimated state covariance
%   - z: current measurement
%   - Q: process noise covariance
%   - R: measurement noise covariance
% Outputs:
%   - x_posterior: a posteriori state estimate
%   - P_posterior: a posteriori state covariance

function [x_posterior,P_posterior] = VanillaEKF(anchor_pos,x_posterior,P_posterior,dT)
%% Symbolic values

    syms p_x p_y p_z % position
    syms v_x v_y v_z % velocity
    x = [p_x,v_x,p_y,v_y,p_z,v_z];

%% Process and measurement model

    % system matrix for constant velocity model
    A = [1,dT,0,0,0,0;0,1,0,0,0,0; ...
        0,0,1,dT,0,0;0,0,0,1,0,0; ...
        0,0,0,0,1,dT;0,0,0,0,0,1];
    
    % non-linear measurement prediction model
    index = 1; 
    r = sym(zeros(size(anchor_pos,1),1));
    for i = 1:size(anchor_pos,1)
        r(index) = sqrt((x(1)-anchor_pos(i,1))^2+(x(3)-anchor_pos(i,2))^2+(x(5)-anchor_pos(i,3))^2);
        index = index + 1;
    end
    
    % measurement matrix
    H = jacobian(r,x);
    H = convertToAcceptArray(H);
    
    % process noise covariance
    dQ = [1/4*dT^4,1/2*dT^3; ...
        1/2*dT^3,dT^2]*q_sigma^2;
    Q = [dQ,zeros(2,4);zeros(2,2),dQ,zeros(2,4); ...
        zeros(2,4),dQ];
    
    % measurement noise covariance
    
%% Prior update

    x_prior = A*x';
    P_prior = A*P*A'+Q;
    
%% A posteriori update
    
    z = getRangeMeasurement(serial);
    K = P_prior*H'*(H*P*H'+R)^(-1);
    x_posterior = x_prior+K*(z-H*x_prior);
    P_posterior = (eye(6)-K*H)*P_prior;
end
   