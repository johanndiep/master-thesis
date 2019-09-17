% Johann Diep (jdiep@student.ethz.ch) - September 2019
%
% Calculates the circular condition for "fmincon" MATLAB function. For more
% informations, check out
% https://ch.mathworks.com/help/optim/ug/fmincon.html.
%
% Input:
%   - p: Pseudo-input data in form (3 x m)
%   - MidPoint: Midpoint of circular trajectory in form (1 x 2)
%   - Radius: Radius of circle
% 
% Output:
%   - c: Array of nonlinear inequality constraints
%   - ceq: Array of nonlinear equality constraints

function [c,ceq] = CircleCon(p,MidPoint,Radius)
    c = [];
    ceq = norm(vecnorm(p-MidPoint').^2-Radius);
end