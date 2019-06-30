% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% Basic position PID controller.

classdef Controller
    properties
        ProportionalGain
        DifferentialGain
        IntegralGain
        Bebop
        PreviousTime
        PreviousError
        Speed
        ErrorIntegral
        FlightCommand
        ErrorTolerance
    end
    
    methods
        function ControllerObject = Controller()
           tic; % starting the timer at the beginning
           
           ControllerObject.ProportionalGain = 0.05;
           ControllerObject.DifferentialGain = 0.2;
           ControllerObject.IntegralGain = 0;
           ControllerObject.PreviousTime = [0,0,0];
           ControllerObject.PreviousError = [0,0,0];               
           ControllerObject.Speed = 1;
           ControllerObject.ErrorIntegral = [0,0,0];
           ControllerObject.FlightCommand = [0,0,0,0,0,0];
           ControllerObject.ErrorTolerance = 0.1;
           
           ControllerObject.Bebop = BebopControl();
        end
        
        % Calculating the proportional, integral and differential error.
        %
        % Input:
        %   - CurrentError: Current error of the desired variable
        %   - Index: Index of the desired variable
        %
        % Output:
        %   - CumulativeError: Cummulative and scaled error       
        
        function CumulativeError = CalculateError(ControllerObject,CurrentError,Index)
            CurrentTime = toc;
            
            DeltaT = 0;
            if ControllerObject.PreviousTime(Index) ~= 0
               DeltaT = CurrentTime-ControllerObject.PreviousTime(Index); 
            end
            
            DeltaError = CurrentError-ControllerObject.PreviousError(Index);
            
            ErrorProportional = CurrentError
            ControllerObject.ErrorIntegral(Index) = ControllerObject.ErrorIntegral(Index)+CurrentError*DeltaT;
            ErrorDifferential = 0;
            if DeltaT > 0
                ErrorDifferential = DeltaError/DeltaT;
            end
            ControllerObject.PreviousTime(Index) = CurrentTime;
            ControllerObject.PreviousError(Index) = CurrentError;
            
            CumulativeError = ControllerObject.ProportionalGain*ErrorProportional+ControllerObject.DifferentialGain*ErrorDifferential+ControllerObject.IntegralGain*ControllerObject.ErrorIntegral(Index);
        end
        
        % Position controller, corrects only positional displacement.
        %
        % Input:
        %   - CurrentPosition: Current position of the drone
        %   - TargetPosition: Goal position of the drone
        
        function NoTurnFlight(ControllerObject,CurrentPosition,TargetPosition)
            for i = 1:size(CurrentPosition,1)
                ControllerObject.FlightCommand(i) = ControllerObject.CalculateError(TargetPosition(i)-CurrentPosition(i),i);
            end
            
            if max(ControllerObject.FlightCommand) > ControllerObject.ErrorTolerance || min(ControllerObject.FlightCommand) < -ControllerObject.ErrorTolerance
                ControllerObject.Bebop.MovementCommand([ControllerObject.Speed*ControllerObject.FlightCommand(1),ControllerObject.Speed*ControllerObject.FlightCommand(2),ControllerObject.Speed*ControllerObject.FlightCommand(3),0,0,0]);
            end 
        end
        
        function Start(ControllerObject)
            ControllerObject.Bebop.TakeOffCommand;
        end
        
        function End(ControllerObject)
            ControllerObject.Bebop.LandCommand;
        end
    end
end
