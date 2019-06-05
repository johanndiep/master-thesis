% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This is an implementation of an extended Kalman Filter for nonlinear
% dynamics systems. The function returns the state estimate x and
% covariance P for nonlinear dynamics system of form:
%   - x_k = q(x_k-1,v_k-1), v ~ N(0,Q)
%   - z_k = h(x_k,w_k), w ~ N(0,R)
% Inputs:
%   - q: nonlinear process function
%   - h: nonlinear measurement function
%   - x: a priori state estimate
%   - P: a priori estimated state covariance
%   - z: current measurement
%   - Q: process noise covariance
%   - R: measurement noise covariance
% Outputs:
%   - x: a posteriori state estimate
%   - P: a posteriori state covariance

function [x,P] = ekfEstimation(q,x,P,h,z,Q,R)
    

end

function A = jaccsd(fun,x)
    
end