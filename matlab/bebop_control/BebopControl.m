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
        
        function PController(BebopObject,ReferencePosition,ActualPosition)
            Error = ReferencePosition - ActualPosition;
            Error = Error/norm(Error);
            BebopObject.MovementCommand([Error(1),Error(2),Error(3),0,0,0]);
        end
    end
end