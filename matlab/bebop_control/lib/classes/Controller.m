% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Basic position and velocity PD controller for the Bebop drone.

classdef Controller
    properties
        P
        Ph
        Py
        D
        Dh
        TreshYaw
        Publisher
    end
    
    methods
        % Initialize the control object with the publisher object 
        % for messaging, the chosen gains and the threshold for yaw-rotations. 
        function ControlObject = Controller()
           ControlObject.Publisher = BebopControl();
 
           ControlObject.P = 0.1; % forward/backward, left/right
           ControlObject.Ph = 0.3; % ascend/descend
           ControlObject.Py = 2; % yaw-rotation
           ControlObject.D = 0.15; % forward/backward, left/right
           ControlObject.Dh = 0; % ascend/descend
           
           ControlObject.TreshYaw = 1/180*pi;
        end
        
        % Calculating the translative proportional and differential error.
        %   - ControlObject: Controller object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function TransError = CalcTransError(ControlObject,CurPos,GoalPos,CurVel,GoalVel,CurQuat)      
            P = ControlObject.P;
            Ph = ControlObject.Ph;
            D = ControlObject.D;
            Dh = ControlObject.Dh;
            
            CurRot = quat2rotm(CurQuat')';
            PosErr = CurRot*(GoalPos-CurPos);
            VelErr = CurRot*(GoalVel-CurVel);
            
            Pc = [P,P,Ph].*PosErr';
            Dc = [D,D,Dh].*VelErr';
            
            TransError = Pc+Dc;
        end
        
        % Calculating the rotational proportional error
        %   - ControlObject: Controller object defined by the constructor
        %   - CurYaw: Current yaw angle in scalar form
        function YawError = CalcYawError(ControlObject,CurYaw)
            Py = ControlObject.Py;
            
            Pc = -Py*CurYaw;
            
            YawError = Pc;
        end
        
        % Position controller which corrects for positional displacement from 
        % goal position. Thereby, the yaw angle is fixed.
        %   - ControlObject: Controller object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion (qw,wx,qy,qz) form (4 x 1)
        function NoTurnFlight(ControlObject,CurPos,GoalPos,CurVel,GoalVel,CurQuat)
            CurOrient = quat2eul(CurQuat');
            CurYaw = CurOrient(1);
            
            TransError = ControlObject.CalcTransError(CurPos,GoalPos,CurVel, ...
                GoalVel,CurQuat);
            
            if abs(CurYaw) > ControlObject.TreshYaw
                YawError = ControlObject.CalcYawError(CurYaw);
                
                % first correct translational error, then rotational error
                ControlObject.Publisher.LinearCommand(TransError);
                ControlObject.Publisher.AngularCommand(YawError);
            else
                ControlObject.Publisher.LinearCommand(TransError);
            end
        end
    
        % Starting takeoff by slowly increasing motor speeds.
        %   - ControlObject: Controller object defined by the constructor
        function Start(ControlObject)
            ControlObject.Publisher.TakeOffCommand;
        end
        
        % Save landing by slowly decreasing motor speeds.
        %   - ControlObject: Controller object defined by the constructor
        function End(ControlObject)
            ControlObject.Publisher.LandCommand;
        end

        % Immediately shutting down all motors.
        %   - ControlObject: Controller object defined by the constructor
        function Emergency(ControlObject)
            ControlObject.Publisher.EmergencyCommand;
        end
    end
end
