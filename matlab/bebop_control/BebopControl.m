% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This class stores all the methods regarding Bebop drone control. 

classdef BebopControl
    properties
        TakeOff
        Land
        Emergency
        Flight
    end
    
    methods
        % Initializing the publisher object with the corresponding ROS
        % message. For more information, check out 
        % https://bebop-autonomy.readthedocs.io/en/latest/.
        function Publisher = BebopControl()
           Publisher.TakeOff = rospublisher('/bebop/takeoff','std_msgs/Empty');
           Publisher.Land = rospublisher('/bebop/land','std_msgs/Empty');
           Publisher.Emergency = rospublisher('/bebop/reset','std_msgs/Empty');
           Publisher.Flight = rospublisher('/bebop/cmd_vel','geometry_msgs/Twist');
        end
        
        % Starting takeoff by increasing motor speeds.
        %   - Publisher: Publisher object defined by the constructor
        function TakeOffCommand(Publisher)
            TakeOffMessage = rosmessage(Publisher.TakeOff);
            send(Publisher.TakeOff,TakeOffMessage);
        end
        
        % Save landing by decreasing motor speeds.
        %   - Publisher: Publisher object defined by the constructor
        function LandCommand(Publisher)
            LandMessage = rosmessage(Publisher.Land);
            send(Publisher.Land,LandMessage);
        end
        
        % Immediately shutting down all motors.
        %   - Publisher: Publisher object defined by the constructor
        function EmergencyCommand(Publisher)
            EmergencyMessage = rosmessage(Publisher.Emergency);
            send(Publisher.Emergency,EmergencyMessage);
        end
        
        % For basic translational movements in each direction as well as 
        % angular movements around each axis. For more information, check 
        % out https://bebop-autonomy.readthedocs.io/en/latest/.
        %   - Publisher: Publisher object defined by the constructor
        %   - FlightCommand: Array holding the commands for translation and rotation 
        %     in the form [1,6]
        function MovementCommand(Publisher,FlightCommand)
            MovementMessage = rosmessage(Publisher.Flight);
            MovementMessage.Linear.X = FlightCommand(1);
            MovementMessage.Linear.Y = FlightCommand(2);
            MovementMessage.Linear.Z = FlightCommand(3);
            MovementMessage.Angular.X = FlightCommand(4);
            MovementMessage.Angular.Y = FlightCommand(5);
            MovementMessage.Angular.Z = FlightCommand(6);
            send(Publisher.Flight,MovementMessage);
        end
    end
end