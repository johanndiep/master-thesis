% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This class offers multiple methods for generating specific trajectories:
%   - Static hovering over a specified position while fixing yaw
%   - Circular flight with specified midpoint and radius while fixing yaw
%   - Circular flight with specified midpoint and radius while heading at
%     the direction of flight

classdef TrajectoryGenerator < handle
    properties
        m
        h
        v
        r
        f
        s
        FullRot
    end
    
    methods
        % Initialize the trajectory object with the chosen parameters.
        %   - MidPoint: Midpoint of circular trajectory in form (1 x 2)
        %     for simple position control the midpoint defines the desired goal position 
        %   - Height: Constant height of the trajectory in scalar form
        %   - AbsVel: Constant velocity in scalar form
        %   - Radius: Radius of circle
        %   - Frequency: Rate for the change of waypoints
        %   - SplinePoints: Points for spline approximation in form (n x 3)
        function TrajectoryObject = TrajectoryGenerator(MidPoint,Height,AbsVel,Radius,Frequency,SplinePoints)
            if nargin == 3
               Radius = 0;
               Frequency = 0;
               SplinePoints = 0;
            elseif nargin == 5
               SplinePoints = 0;
            end
            
            TrajectoryObject.m = MidPoint;
            TrajectoryObject.h = Height;
            TrajectoryObject.v = AbsVel;
            TrajectoryObject.r = Radius;
            TrajectoryObject.f = Frequency;
            TrajectoryObject.s = SplinePoints;
            
            TrajectoryObject.FullRot = 2*pi;
        end
        
        % This function calculates the dynamic position and the corresponding 
        % velocity for a circular flight, where the drone keeps facing
        % forward.
        %   - TrajectoryObject: Trajectory object defined by the constructor
        %   - t: Timestep at which the the corresponding waypoint should be
        %        gathered
        function [GoalPos,GoalVel] = getCircleTrajectory(TrajectoryObject,t)
            m = TrajectoryObject.m;
            h = TrajectoryObject.h;
            v = TrajectoryObject.v;
            r = TrajectoryObject.r;
            f = TrajectoryObject.f;
            
            GoalPos(1) = m(1)+r*sin(2*pi*f*t);
            GoalPos(2) = m(2)-r*cos(2*pi*f*t);
            GoalPos(3) = h;
    
            GoalVel(1) = v*cos(2*pi*f*t);
            GoalVel(2) = v*sin(2*pi*f*t);
            GoalVel(3) = 0;       
        end

        % This function calculates the dynamic position and the corresponding 
        % velocity for a circular flight, where the drone is facing its flying
        % direction.
        %   - TrajectoryObject: Trajectory object defined by the constructor
        %   - t: Timestep at which the the corresponding waypoint should be
        %        gathered        
        function [GoalPos,GoalYaw,GoalVel] = getYawCircleTraj(TrajectoryObject,t)
            m = TrajectoryObject.m;
            h = TrajectoryObject.h;
            v = TrajectoryObject.v;
            r = TrajectoryObject.r;
            f = TrajectoryObject.f;
            FullRot = TrajectoryObject.FullRot;
            
            GoalPos(1) = m(1)+r*sin(2*pi*f*t);
            GoalPos(2) = m(2)-r*cos(2*pi*f*t);
            GoalPos(3) = h;

            Yaw = 2*pi*f*t;
            RotationCounter = floor(Yaw/FullRot);
            GoalYaw = Yaw - RotationCounter*2*pi;
    
            GoalVel(1) = v*cos(2*pi*f*t);
            GoalVel(2) = v*sin(2*pi*f*t);
            GoalVel(3) = 0;
        end
             
        % This function calculates the dynamic position and the corresponding 
        % velocity for a circular flight, where the drone is facing to the
        % center.
        %   - TrajectoryObject: Trajectory object defined by the constructor
        %   - t: Timestep at which the the corresponding waypoint should be
        %        gathered        
        function [GoalPos,GoalYaw,GoalVel] = getCenCircleTraj(TrajectoryObject,t)
            m = TrajectoryObject.m;
            h = TrajectoryObject.h;
            v = TrajectoryObject.v;
            r = TrajectoryObject.r;
            f = TrajectoryObject.f;
            FullRot = TrajectoryObject.FullRot;
            
            GoalPos(1) = m(1)+r*sin(2*pi*f*t);
            GoalPos(2) = m(2)-r*cos(2*pi*f*t);
            GoalPos(3) = h;

            Yaw = 2*pi*f*t+0.5*pi;
            RotationCounter = floor(Yaw/FullRot);
            GoalYaw = Yaw - RotationCounter*2*pi;
    
            GoalVel(1) = v*cos(2*pi*f*t);
            GoalVel(2) = v*sin(2*pi*f*t);
            GoalVel(3) = 0;
        end        
        
        % This function returns a static position for hovering.
        %   - TrajectoryObject: Trajectory object defined by the constructor 
        function [GoalPos,GoalVel] = getStaticPosition(TrajectoryObject)
            m = TrajectoryObject.m;
            h = TrajectoryObject.h;
            v = TrajectoryObject.v;
            
            GoalPos(1) = m(1);
            GoalPos(2) = m(2);
            GoalPos(3) = h;
            
            GoalVel(1) = v;
            GoalVel(2) = v;
            GoalVel(3) = v;            
        end
        
        % This function calculates the dynamic position and the corresponding 
        % velocity for a spline flight, where the drone is facing its flying
        % direction.
        %   - TrajectoryObject: Trajectory object defined by the constructor
        %   - t: Timestep at which the the corresponding waypoint should be
        %        gathered
        function [GoalPos,GoalYaw,GoalVel] = getSplinePosition(TrajectoryObject,t)
            h = TrajectoryObject.h;
            f = TrajectoryObject.f;
            s = TrajectoryObject.s;
            
            xPoints = s(:,1); yPoints = s(:,2);
            
            u = [0,1,2,3,4,5];
            uq = 0:0.05:5;
            
            SlopeStart = 0; SlopeFinal = 0;
            
            xQuery = spline(u,[SlopeStart;xPoints;SlopeFinal],uq);
            yQuery = spline(u,[SlopeStart;yPoints;SlopeFinal],uq);
            
            AbsVel = getArcLength(xQuery,yQuery,'s')*f;
            
            N = size(xQuery,2);
            CurrentIndex = ceil(N*f*t);
            NextIndex = CurrentIndex+1;
            if CurrentIndex >= N
                CurrentIndex = N-1;
                NextIndex = N;
            end
            
            GoalPos(1) = xQuery(CurrentIndex);
            GoalPos(2) = yQuery(CurrentIndex);
            GoalPos(3) = h;
            
            xDiff = xQuery(NextIndex)-xQuery(CurrentIndex);
            yDiff = yQuery(NextIndex)-yQuery(CurrentIndex);
            Norm = vecnorm([xDiff,yDiff]);
            
            GoalYaw = atan2(yDiff,xDiff);
            if GoalYaw < 0
               GoalYaw = 2*pi+GoalYaw; 
            end
            
            GoalVel(1) = xDiff/Norm*AbsVel;
            GoalVel(2) = yDiff/Norm*AbsVel;
            GoalVel(3) = 0;
        end
    end
end