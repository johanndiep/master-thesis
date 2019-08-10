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
        % Initialize the parameter object with the chosen gains and the
        % publisher object for messaging. 
        function Parameters = Controller()
           Parameters.P = [0.2,0.2,0.2];
           Parameters.D = [0,0,0];
           Parameters.TreshRot = 0.05; % roughly 3 degree deviation
           Parameters.Publisher = BebopControl();
        end
        
        % Calculating the cummulative proportional and differential error.
        %   - Parameters: Parameter object defined by the constructor
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)

        function CumulativeError = CalculateError(Parameters,CurPos,GoalPos,CurVel,GoalVel,CurQuat)
            
            CurRot = quat2rotm(CurQuat)';
            PosErr = CurRot*(GoalPos-CurPos);
            VelErr = CurRot*(GoalVel-CurVel);
            
            Pd = Parameters.P.*PosErr';
            Dd = Parameters.D.*VelErr';
            
            CumulativeError = Pd+Dd;
        end
        
        % Position controller which corrects for positional displacement from 
        % goal position. Thereby, the yaw angle is fixed.
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurQuat: Current orientation in quaternion form (4 x 1)
        function NoTurnFlight(Parameters,CurPos,GoalPos,CurVel,GoalVel,CurQuat)
            CurOrient = quat2eul(CurQuat');
            CurYaw = CurOrient(1);
            
            if abs(CurYaw) > Parameters.TreshRot
                Parameters.Publisher.AngularCommand(-1*CurYaw);
            else
                CumulativeError = Parameters.CalculateError(GoalPos,CurPos,GoalPos,CurVel,GoalVel,CurQuat);
                Parameters.Publisher.LinearCommand(CumulativeError);
            end
        end
    
        % Starting takeoff by increasing motor speeds.
        %   - Publisher: Publisher object defined by the constructor
        function Start(Parameters)
            Parameters.Publisher.TakeOffCommand;
        end
        
        % Save landing by decreasing motor speeds.
        %   - Parameters: Parameter object defined by the constructor
        function End(Parameters)
            Parameters.Publisher.LandCommand;
        end

        % Immediately shutting down all motors.
        %   - Parameters: Parameter object defined by the constructor
        function Emergency(Parameters)
            Parameters.Publisher.EmergencyCommand;
        end
    end
end
