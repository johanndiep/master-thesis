% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This function calculates the waypoints for a circular flight.
%
% Input:
%   - MidPoint: Midpoint of circular trajectory in form (1 x 2)
%   - Height: Constant height of the trajectory
%   - Radius: Radius of circle
%   - f: Rate for the change of waypoints
%   - t: Timestep at which the the corresponding waypoint should be
%        gathered
%
% Output:
%   - DesPos: Desired waypoint
%   - DesVel: Corresponding desired velocity

function [DesPos,DesVel] = getCircleTrajectory(MidPoint,Height,Radius,f,t)
    AbsVel = 0.1;

    DesPos(1) = MidPoint(1)+Radius*sin(2*pi*f*t);
    DesPos(2) = MidPoint(2)-Radius*cos(2*pi*f*t);
    DesPos(3) = Height;
    
    DesVel(1) = AbsVel*cos(2*pi*f*t);
    DesVel(2) = AbsVel*sin(2*pi*f*t);
    DesVel(3) = 0;
end