% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This script allows the control of the Bebop drone with the 3D connexion
% joystick. It also collects VICON measurements for ground-truth data.
%
% Step-by-Step:
%   1. Turn on the Spacemouse and start its ROS driver.
%   2. Attach VICON markers on the Bebop, group the markers on the VICON
%      Tracker to an object and name it "Bebop_Johann".
%   3. Place the drone such that the body-fixed frame (x-forward,y-left,z-ascend)
%      is aligned with the VICON frame.
%   4. Connect the computer with the VICON machine via Ethernet.
%   5. Turn on the Bebop and connect the laptop with it over Wi-Fi.
%   6. Start the ROS driver for the Bebop.
%   7. Start the ROS VICON bridge node.
%   8. Adjust the variable "DominantFlight" to either 1 or 0. With dominant
%      flight mode set to 1, the drone only moves in the direction with the largest
%      offset.
%   9. Run the following script.
%   10. Control the drone by steering the joystick.

clear; clc;

rosshutdown; rosinit;

%% Joystick Control

JoySubscriber = rossubscriber('/spacenav/joy');
VicDroneSub = rossubscriber('/vicon/Bebop_Johann/Bebop_Johann');

BebopPublisher = BebopControl();

FlyState = 0;
DominantFlight = 1; % only fly in the main direction/rotation
Index = 1;
Save = true;

while true
    JoyMessage = JoySubscriber.LatestMessage;
    
    if JoyMessage.Buttons(1) == 1
        BebopPublisher.TakeOffCommand;
        pause(5);
        tic;
        FlyState = 1;
    elseif JoyMessage.Buttons(2) == 1
        T = toc;
        BebopPublisher.LandCommand;
        FlyState = 0;
        break;
    end
    
    if FlyState == 1
        [ViconPos(:,Index),ViconQuat(:,Index)] = getGroundTruth(VicDroneSub);
        JoyCommand = JoyMessage.Axes';
        JoyCommand(4:5) = [];
        
        if DominantFlight == 1
            [MaxValue,MaxIndex] = max(JoyCommand);
            [MinValue,MinIndex] = min(JoyCommand);
            FlightCommand(Index,:) = zeros(1,4);
            
            if abs(MaxValue) > abs(MinValue)
                FlightCommand(Index,MaxIndex) = MaxValue;
                BebopPublisher.MovementCommand(FlightCommand(Index,:));
                disp(FlightCommand(Index,:));
            else
                FlightCommand(Index,MinIndex) = MinValue;
                BebopPublisher.MovementCommand(FlightCommand(Index,:));
                disp(FlightCommand(Index,:));
            end
        else
            FlightCommand(Index,:) = JoyCommand;
            BebopPublisher.MovementCommand(FlightCommand(Index,:));
            disp(FlightCommand(Index,:));
        end
        
        Index = Index+1;
    end  
end

rosshutdown;

if Save == true
    FlightCommand(:,4) = [];
    save('FlightCommand.mat','FlightCommand','ViconPos','ViconQuat','T');
end