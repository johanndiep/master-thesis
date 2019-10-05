% Johann Diep (jdiep@student.ethz.ch) - October 2019
%
% Contains the dynamic models of the Bebop drone.

classdef DynamicModel < handle
    properties
        g
        C
        MaxTiltAngle
        MaxVertSpeed
        MaxRotSpeed
    end
    
    methods
        % Sets the physical parameters of the drone as well as the
        % environment.
        function DynamicObject = DynamicModel()
            DynamicObject.g = 9.81; % gravitational constant
            DynamicObject.C = 0.35; % translational drag constant
            DynamicObject.MaxTiltAngle = 15/180*pi; % maximal pitch and roll angle
            DynamicObject.MaxVertSpeed = 1; % maximal speed in vertical direction
        end
        
        % This function calculates the new state from the current state using the
        % discretized and simplified Bebop drone model. The pitch and yaw
        % angles are estimated as step functions. The yaw angle is ignored.
        %   - DynamicObject: Dynamic object defined by the constructor
        %   - u: Input between (-1,1) for (Theta,Phi,vz) in form (1,3)
        %   - CurState: Current state in form (x,vx,y,vy,z,vz)
        %   - dT: Time interval between each  iteration
        function [NewState] = BebopDisSimModel(DynamicObject,u,CurState,dT)
            g = DynamicObject.g;
            MaxTiltAngle = DynamicObject.MaxTiltAngle;
            MaxVertSpeed = DynamicObject.MaxVertSpeed;
            
            Theta = u(1)*MaxTiltAngle;
            Phi = u(2)*MaxTiltAngle;
            Vz = u(3)*MaxVertSpeed;
            
            NewState(1) = CurState(1)+dT*CurState(2)+0.5*dT^2*g*tan(Theta);
            NewState(2) = CurState(2)+dT*g*tan(Theta);
            NewState(3) = CurState(3)+dT*CurState(4)+0.5*dT^2*g*tan(Phi);
            NewState(4) = CurState(4)+dT*g*tan(Phi);
            NewState(5) = CurState(5)+dT*Vz;
            NewState(6) = Vz;
        end
        
        % This function calculates the new state from the current state
        % using the discretized and full Bebop drone model, where the translational 
        % drag effects are also considered. The pitch and yaw angles are estimated 
        % as step functions. The yaw angle is ignored.
        %   - DynamicObject: Dynamic object defined by the constructor
        %   - u: Input between (-1,1) for (Theta,Phi,vz) in form (1,3)
        %   - CurState: Current state in form (x,vx,y,vy,z,vz)
        %   - dT: Time interval between each  iteration
        function [NewState] = BebopDiscreteModel(DynamicObject,u,CurState,dT)
            g = DynamicObject.g;
            C = DynamicObject.C;
            MaxTiltAngle = DynamicObject.MaxTiltAngle;
            MaxVertSpeed = DynamicObject.MaxVertSpeed;
            
            Theta = u(1)*MaxTiltAngle;
            Phi = u(2)*MaxTiltAngle;
            Vz = u(3)*MaxVertSpeed;
            
            NewState(1) = CurState(1)+1/C*CurState(2)-1/C^2*g*tan(Theta)- ...
                1/C^2*exp(-C*dT)*(C*CurState(2)-g*tan(Theta))+ ...
                1/C*g*dT*tan(Theta);
            NewState(2) = 1/C*exp(-C*dT)*(C*CurState(2)-g*tan(Theta))+ ...
                1/C*g*tan(Theta);
            NewState(3) = CurState(3)+1/C*CurState(4)-1/C^2*g*tan(Phi)- ...
                1/C^2*exp(-C*dT)*(C*CurState(4)-g*tan(Phi))+ ...
                1/C*g*dT*tan(Phi);            
            NewState(4) = 1/C*exp(-C*dT)*(C*CurState(4)-g*tan(Phi))+ ...
                1/C*g*tan(Phi);
            NewState(5) = CurState(5)+dT*Vz;
            NewState(6) = Vz;
        end
    end
end