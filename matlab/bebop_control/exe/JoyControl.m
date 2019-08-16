% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script allows the control of the Bebop drone with the 3D connexion
% joystick. Before starting this script, run the following command in
% terminal: 
%   1. Start the ROS driver for the Spacemouse
%   2. Start the ROS driver for the Bebop
%   3. Run the following script

clear;
clc;

rosshutdown; rosinit;

%% Joystick Control

JoySubscriber = rossubscriber('/spacenav/joy');
BebopPublisher = BebopControl();

FlyState = 0;
DominantFlight = 1; % only fly in the main direction/rotation

while true
    JoyMessage = JoySubscriber.LatestMessage;
    
    if JoyMessage.Buttons(1) == 1
        BebopPublisher.TakeOffCommand;
        FlyState = 1;
    elseif JoyMessage.Buttons(2) == 1
        BebopPublisher.LandCommand;
        FlyState = 0;
        break;
    end
    
    if FlyState == 1
        JoyCommand = JoyMessage.Axes';
        JoyCommand(4:5) = [];
        
        if DominantFlight == 1
            [MaxValue,MaxIndex] = max(JoyCommand);
            [MinValue,MinIndex] = min(JoyCommand);
            FlightCommand = zeros(1,4);
            
            if abs(MaxValue) > abs(MinValue)
                FlightCommand(MaxIndex) = MaxValue;
                BebopPublisher.MovementCommand(FlightCommand);
                % disp(FlightCommand);
            else
                FlightCommand(MinIndex) = MinValue;
                BebopPublisher.MovementCommand(FlightCommand);
                % disp(FlightCommand);
            end
        else
            FlightCommand = JoyCommand;
            BebopPublisher.MovementCommand(FlightCommand);
            % disp(FlightCommand);
        end
    end  
end

rosshutdown;