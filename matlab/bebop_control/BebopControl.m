% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This class stores all the methods regarding Bebop drone control.

classdef BebopControl
    properties
        TakeOffPublisher
        LandPublisher
        EmergencyPublisher
        FlightPublisher
    end
    
    methods
        function BebopObject = BebopControl()
           BebopObject.TakeOffPublisher = rospublisher('/bebop/takeoff','std_msgs/Empty');
           BebopObject.LandPublisher = rospublisher('/bebop/land','std_msgs/Empty');
           BebopObject.EmergencyPublisher = rospublisher('/bebop/reset','std_msgs/Empty');
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
        
        function EmergencyCommand(BebopObject)
            EmergencyMessage = rosmessage(BebopObject.EmergencyPublisher);
            send(BebopObject.EmergencyPublisher,EmergencyMessage);
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
    end
end