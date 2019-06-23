% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This class stores all the methods regarding Bebop drone control.

classdef BebopControl
    properties
        TakeOffPublisher
        LandPublisher
        FlightPublisher
    end
    
    methods
        function BebopObject = BebopControl()
           BebopObject.TakeOffPublisher = rospublisher('/bebop/takeoff','std_msgs/Empty');
           BebopObject.LandPublisher = rospublisher('/bebop/land','std_msgs/Empty');
           BebopObject.FlightPublisher = rospublisher('/bebop/cmd_vel','geometry_msgs/Twist');
        end
        
        function TakeOffCommand(BebopObject)
            TakeOffMessage = rosmessage(BebopObject.TakeOffPublisher);
            send(BebopObject.TakeOffPublisher,TakeOffMessage);
        end
        
        function LandCommand(BebopObject)
            LandMessage = rosmessage(BebopObject.LandPublisher);
            send(BebopObject.LandPublisher,LandMessage);
        end
        
        % For basic translational movements in each direction as well as 
        % angular movements around each axis. For more information, check 
        % out https://bebop-autonomy.readthedocs.io/en/latest/.
        %
        % Input:
        %   - FlightCommand: Array holding the commands for translation and rotation in the form [1,6]    
        function MovementCommand(BebopObject,FlightCommand)
            MovementMessage = rosmessage(BebopObject.FlightPublisher);
            MovementMessage.Linear.X = FlightCommand(1);
            MovementMessage.Linear.Y = FlightCommand(2);
            MovementMessage.Linear.Z = FlightCommand(3);
            MovementMessage.Angular.X = FlightCommand(4);
            MovementMessage.Angular.Y = FlightCommand(5);
            MovementMessage.Angular.Z = FlightCommand(6);
            send(BebopObject.FlightPublisher,MovementMessage);
        end
        
        % Basic position PID controller.
        %
        % Input: 
        %   - ReferencePosition: Desired position in World-frame in the form [3,1]
        %   - ActualPosition: Current position in World-frame in the form [3,1]
        function PController(BebopObject,ReferencePosition,ActualPosition)
            Error = ReferencePosition - ActualPosition;
            Error = Error/norm(Error);
            BebopObject.MovementCommand([Error(1),Error(2),Error(3),0,0,0]);
        end
    end
end