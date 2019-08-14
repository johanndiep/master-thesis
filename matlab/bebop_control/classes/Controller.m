% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Basic position PD controller for the Bebop drone.

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
           ControlObject.TreshRot = 0.1; % roughly 6 degree deviation
           ControlObject.Publisher = BebopControl();
        end
        
        % Calculating the cummulative proportional and differential error.
        %   - Parameters: Parameter object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function CumulativeError = CalculateError(ControlObject,CurPos,GoalPos,CurVel, ...
                GoalVel,CurQuat)
            
            CurRot = quat2rotm(CurQuat')';
            PosErr = CurRot*(GoalPos-CurPos);
            VelErr = CurRot*(GoalVel-CurVel);
            
            Pd = ControlObject.P*PosErr';
            Dd = ControlObject.D*VelErr';
            
            CumulativeError = Pd+Dd;
        end
        
        % Position controller which corrects for positional displacement from 
        % goal position. Thereby, the yaw angle is fixed.
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function NoTurnFlight(ControlObject,CurPos,GoalPos,CurVel,GoalVel,CurQuat)
            CurOrient = quat2eul(CurQuat');
            CurYaw = CurOrient(1);
            
            if abs(CurYaw) > ControlObject.TreshRot
                ControlObject.Publisher.AngularCommand(-1*CurYaw);
            else
                CumulativeError = ControlObject.CalculateError(GoalPos,CurPos,GoalPos,CurVel, ...
                    GoalVel,CurQuat);
                ControlObject.Publisher.LinearCommand(CumulativeError);
            end
        end
    
        % Starting takeoff by increasing motor speeds.
        %   - Publisher: Publisher object defined by the constructor
        function Start(ControlObject)
            ControlObject.Publisher.TakeOffCommand;
        end
        
        % Save landing by decreasing motor speeds.
        %   - Parameters: Parameter object defined by the constructor
        function End(ControlObject)
            ControlObject.Publisher.LandCommand;
        end

        % Immediately shutting down all motors.
        %   - Parameters: Parameter object defined by the constructor
        function Emergency(ControlObject)
            ControlObject.Publisher.EmergencyCommand;
        end
    end
end
