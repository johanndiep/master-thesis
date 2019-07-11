% Johann Diep (jdiep@student.ethz.ch) - July 2019
%
% This function executes sinusoidal curve fitting.
%
% Input:
%   - RotationAngles: Angle of the tag node
%   - ErrorArray: Corresponding error offset
% 
% Output:
%   - SinusoidalCoefficients: Coefficients for the sinusoidal fitting curve
%   - SinusoidalFunction: Function handle for the sinusoidal fitting curve

function [SinusoidalCoefficients,SinusoidalFunction] = SinusoidalFit(RotationAngles,ErrorArray)
    ErrorArray = ErrorArray';
    RotationAngles = RotationAngles';
    
    SinusoidalFunction = @(p,x) p(1)*(sin(2*pi/p(2)*x + 2*pi/p(3)))+p(4);
    ObjectiveFunction = @(p) sum((SinusoidalFunction(p,RotationAngles)-ErrorArray).^2);
    SinusoidalCoefficients = fminsearch(ObjectiveFunction,[0.05;180;-1;0.1]);
end
