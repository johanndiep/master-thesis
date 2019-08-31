% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This class stores all the methods regarding Bebop drone control. For more 
% information, check out https://bebop-autonomy.readthedocs.io/en/latest/.

classdef BebopControl
    properties
        TakeOff
        Land
        Emergency
        Flight
    end
    
    methods
        % Initializing the publisher object with the corresponding ROS message.
        function Publisher = BebopControl()
           Publisher.TakeOff = rospublisher('/bebop/takeoff','std_msgs/Empty');
           Publisher.Land = rospublisher('/bebop/land','std_msgs/Empty');
           Publisher.Emergency = rospublisher('/bebop/reset','std_msgs/Empty');
           Publisher.Flight = rospublisher('/bebop/cmd_vel','geometry_msgs/Twist');
        end
        
        % Starting takeoff by slowly increasing motor speeds.
        %   - Publisher: Publisher object defined by the constructor
        function TakeOffCommand(Publisher)
            TakeOffMessage = rosmessage(Publisher.TakeOff);
            send(Publisher.TakeOff,TakeOffMessage);
        end
        
        % Save landing by slowly decreasing motor speeds.
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
        % yaw angular rotations.
        %   - Publisher: Publisher object defined by the constructor
        %   - FlightCommand: Array holding the commands for translation and rotation 
        %     in the form (1 x 4)
        function MovementCommand(Publisher,FlightCommand)
            MovementMessage = rosmessage(Publisher.Flight);
            MovementMessage.Linear.X = FlightCommand(1);
            MovementMessage.Linear.Y = FlightCommand(2);
            MovementMessage.Linear.Z = FlightCommand(3);
            MovementMessage.Angular.X = 0;
            MovementMessage.Angular.Y = 0;
            MovementMessage.Angular.Z = FlightCommand(4);
            send(Publisher.Flight,MovementMessage);
        end
        
        % For translational movements in each direction only.
        %   - Publisher: Publisher object defined by the constructor
        %   - LinearCommand: Array holding the commands for translation in
        %     the form (1 x 3)
        function LinearCommand(Publisher,LinearCommand)
            LinearMessage = rosmessage(Publisher.Flight);
            LinearMessage.Linear.X = LinearCommand(1);
            LinearMessage.Linear.Y = LinearCommand(2);
            LinearMessage.Linear.Z = LinearCommand(3);
            LinearMessage.Angular.X = 0;
            LinearMessage.Angular.Y = 0;
            LinearMessage.Angular.Z = 0;
            send(Publisher.Flight,LinearMessage);
        end
        
        % For yaw angular movements only.
        %   - Publisher: Publisher object defined by the constructor
        %   - AngularCommand: Command for yaw angular rotation
        function AngularCommand(Publisher,AngularCommand)
            AngularMessage = rosmessage(Publisher.Flight);
            AngularMessage.Linear.X = 0;
            AngularMessage.Linear.Y = 0;
            AngularMessage.Linear.Z = 0;
            AngularMessage.Angular.X = 0;
            AngularMessage.Angular.Y = 0;
            AngularMessage.Angular.Z = AngularCommand;
            send(Publisher.Flight,AngularMessage);
        end
    end
end