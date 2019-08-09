% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Basic position PD controller for the Bebop drone.

classdef Controller
    properties
        P
        D
        TreshRot
        Publisher
    end
    
    methods
        % Initialize the parameter object with the chosen gains and the
        % publisher object for messaging. 
        function Parameters = Controller()
           Parameters.P = 0.02;
           Parameters.D = 0;
           Parameters.TreshRot = 0.05;
           Parameters.Publisher = BebopControl();
        end
        
        % Calculating the cummulative proportional and differential error.
        %   - Parameters: Parameter object defined by the constructor
        %   - PosErr: Position error
        %   - VelErr: Velocity Error
        %   - CurYaw: Current yaw orientation
        function CumulativeError = CalculateError(Parameters,PosErr,VelErr,CurYaw)           
            Pd = Parameters.P * PosErr;
            Dd = Parameters.D * VelErr;
            
            CumulativeError = [Pd + Dd,0,0,-1*CurYaw];
        end
        
        % Position controller which corrects for positional displacement from 
        % goal position. Thereby, the yaw angle is fixed.
        %   - CurPos: Current position of the drone in form (3 x 1)
        %   - GoalPos: Goal position of the drone in form (3 x 1)
        %   - CurVel: Current velocity of the drone in form (3 x 1)
        %   - GoalVel: Goal velocity of the drone in form (3 x 1)
        %   - CurYaw: Current yaw orientation 
        function NoTurnFlight(Parameters,CurPos,GoalPos,CurVel,GoalVel,CurYaw)
            CumulativeError = Parameters.CalculateError(GoalPos-CurPos,GoalVel-CurVel,CurYaw);
            
            if CurYaw > Parameters.TreshRot
                Parameters.Publisher.MovementCommand([0,0,0,0,0,CumulativeError(6)]);
            else
                Parameters.Publisher.MovementCommand([CumulativeError(1:3),0,0,0]);
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
