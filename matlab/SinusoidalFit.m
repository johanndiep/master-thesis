% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This script executes sinusoidal curve fitting.

function [SinusoidalCoefficients,SinusoidalFunction] = SinusoidalFit(RotationAngles,ErrorArray)
    ErrorArray = ErrorArray';
    RotationAngles = RotationAngles';
    
    SinusoidalFunction = @(p,x) p(1)*(sin(2*pi/p(2)*x + 2*pi/p(3)))+p(4);
    ObjectiveFunction = @(p) sum((SinusoidalFunction(p,RotationAngles)-ErrorArray).^2);
    SinusoidalCoefficients = fminsearch(ObjectiveFunction,[0.05;180;50;0.1]);
end
