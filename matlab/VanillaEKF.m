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

function [x_posterior,p_posterior] = VanillaEKF(x_prior,P_prior)
    %% Process model
    
    A = [1,dT,0,0,0,0;0,1,0,0,0,0; ...
        0,0,1,dT,0,0;0,0,0,1,0,0; ...
        0,0,0,0,1,dT;0,0,0,0,0,1];
    
    
end