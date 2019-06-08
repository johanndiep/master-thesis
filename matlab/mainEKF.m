% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This program is the main function which is used to call each executable for the EKF method.
% It will guide the user through each steps, starting from anchor setup
% self-calibration through gathering waypoint datas on the Bebop drone with
% the UWB-ranging method. 

clear;
clc;

disp("This program is the main function which is used to call each executable for the EKF method.");
disp("It will guide the user through each steps, starting from anchor setup");
disp("self-calibration through gathering waypoint datas on the Bebop drone with");
disp("the UWB-ranging method.");
disp("*************************************************");

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Setup serial port

port = seriallist;
serial = serial(port);

%% Hardcoded parameters

iterations = 100; % gather 50 position data
index = 1;
height_top = 2.43-0.275; % anchors heights

%% Calling anchor calibration executables

input("Place anchors in the room and press [ENTER]");
input("Change the connected module into Sniffer mode and press [ENTER]");
disp("*************************************************");
disp("Starting self-calibration");

% starting anchor self-calibration procedure
anchor_range_mean = getAnchorRangeMeasurement(serial);
anchor_pos = AnchorCalibration(anchor_range_mean,false);

%% Plotting the anchors

figure()
hold on
title("Flying arena coordinate system");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
grid on

scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);

% for 6 anchors network
line([anchor_pos(1,1),anchor_pos(2,1)],[anchor_pos(1,2),anchor_pos(2,2)], ...
    [anchor_pos(1,3),anchor_pos(2,3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
line([anchor_pos(3,1),anchor_pos(4,1)],[anchor_pos(3,2),anchor_pos(4,2)], ...
    [anchor_pos(3,3),anchor_pos(4,3)],'Color',[.9412,.9412,.9412],'LineWidth',3);
line([anchor_pos(5,1),anchor_pos(6,1)],[anchor_pos(5,2),anchor_pos(6,2)], ...
    [anchor_pos(5,3),anchor_pos(6,3)],'Color',[.9412,.9412,.9412],'LineWidth',3);

for i = 1:size(anchor_pos,1)
    text(anchor_pos(i,1)+0.1,anchor_pos(i,2)+0.1,anchor_pos(i,3)+0.1,"Anchor " + int2str(i));
end

%% Calling the position estimation executables

disp("*************************************************");
disp("Preparing to gather " + iterations + " waypoints");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
disp("*************************************************");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(serial);
clear serial;
port = seriallist;
serial = serial(port);

% initialization of state [p_x,v_x,p_y,v_y,p_z,v_z] and covariance 
% for constant velocity model
x_posteriori = [0,0,0,0,0,0];
P_posteriori = 10*eye(6);

% getting time for batch of range measurements
tic;
getRangeMeasurement(serial);
dT = toc;

while index < iterations + 1
    [x_posteriori,P_posteriori] = VanillaEKF(anchor_pos,x_posteriori, P_posteriori,dT);
end




