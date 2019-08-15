% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Basic position and velocity PD controller for the Bebop drone.

classdef Controller
    properties
        P
        D
        TreshRot
        TreshTran
        Publisher
    end
    
    methods
        % Initialize the control object with the chosen gains and the
        % publisher object for messaging. 
        %   - P/D: PD gains in scalar form
        function ControlObject = Controller()
           ControlObject.P = 0.05;
           ControlObject.D = 0;
           
           ControlObject.TreshRot = 3/180*pi; % roughly 3 degree deviation
           
           ControlObject.Publisher = BebopControl();
        end
        
        % Calculating the translative proportional and differential error.
        %   - ControlObject: Controller object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function TransError = CalcTransError(ControlObject,CurPos,GoalPos,CurVel, ...
                GoalVel,CurQuat)
            
            CurRot = quat2rotm(CurQuat')';
            PosErr = CurRot*(GoalPos-CurPos);
            VelErr = CurRot*(GoalVel-CurVel);
            
            Pd = ControlObject.P*PosErr';
            Dd = ControlObject.D*VelErr';
            
            TransError = Pd+Dd;
        end
        
        % Calculating the rotational proportional error
        %   - ControlObject: Controller object defined by the constructor
        %   - CurYaw: Current yaw angle in scalar form
        function YawError = CalcYawError(ControlObject,CurYaw)
            Pd = -ControlObject.P*CurYaw;
            YawError = Pd;
        end
        
        % Position controller which corrects for positional displacement from 
        % goal position. Thereby, the yaw angle is fixed.
        %   - ControlObject: Controller object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function NoTurnFlight(ControlObject,CurPos,GoalPos,CurVel,GoalVel,CurQuat)
            CurOrient = quat2eul(CurQuat');
            CurYaw = CurOrient(1);
            
            TransError = ControlObject.CalcTransError(CurPos,GoalPos,CurVel, ...
                GoalVel,CurQuat);
            
            if abs(CurYaw) > ControlObject.TreshRot
                YawError = ControlObject.CalcYawError(CurYaw);
                ControlObject.Publisher.AngularCommand(YawError);
                ControlObject.Publisher.LinearCommand(TransError);
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
