% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This program is the main function which is used to call each executable for the EKF method.
% It will guide the user through each steps, starting from anchor setup self-calibration
% through gathering waypoint datas on the Bebop drone with the UWB-ranging method.

clear;
clc;

disp("This program is the main function which is used to call each executable for the EKF method.");
disp("It will guide the user through each steps, starting from anchor setup self-calibration");
disp("through gathering waypoint datas on the Bebop drone with the UWB-ranging method.");
disp("******************************************************************************************************");

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Setup serial port

port = seriallist;
serial = serial(port);

%% Hardcoded parameters

index = 1;
iterations = 100; % gather 50 position data
height_top = 2.43-0.275; % anchors heights
time_iterations = 10;

%% Calling anchor calibration executables

input("Place anchors in the room and press [ENTER]");
input("Change the connected module into Sniffer mode and press [ENTER]");
disp("******************************************************************************************************");
disp("Starting self-calibration");

% starting anchor self-calibration procedure
anchor_range_mean = getAnchorRangeMeasurement(serial);
anchor_pos = AnchorCalibration(anchor_range_mean);

%% Plotting the anchors

figure()
hold on
title("Tinamu Flying Machine Arena");
xlabel("x-Axis [m]");
ylabel("y-Axis [m]");
zlabel("z-Axis [m]");
grid on

scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'MarkerEdgeColor','k','MarkerFaceColor',[0,0,0]);

% for 8 anchors network
% anchor_comb = [1,5;4,8;2,6;3,7];
% for i = 1:size(anchor_comb,1)
%     line([anchor_pos(anchor_comb(i,1),1),anchor_pos(anchor_comb(i,2),1)], ...
%         [anchor_pos(anchor_comb(i,1),2),anchor_pos(anchor_comb(i,2),2)], ...
%         [anchor_pos(anchor_comb(i,1),3),anchor_pos(anchor_comb(i,2),3)], ...
%         'Color',[.9412,.9412,.9412],'LineWidth',3);
% end

% for 6 anchors network
for i = 1:size(anchor_pos,1)
    if mod(i,2) == 1
        line([anchor_pos(i,1),anchor_pos(i+1,1)], ...
            [anchor_pos(i,2),anchor_pos(i+1,2)], ...
            [anchor_pos(i,3),anchor_pos(i+1,3)], ...
            'Color',[.9412,.9412,.9412],'LineWidth',3);
   end
end

for i = 1:size(anchor_pos,1)
    text(anchor_pos(i,1)+0.1,anchor_pos(i,2)+0.1,anchor_pos(i,3)+0.1,"Anchor "+int2str(i));
end

%% Calling the position estimation executables

("******************************************************************************************************");
disp("Preparing to gather " + iterations + " waypoints");
input("Change the module on the Bebop drone into Tag mode and press [ENTER]");
("******************************************************************************************************");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(serial);
clear serial;
port = seriallist;
serial = serial(port);

% initialization of state [p_x,p_y,p_z,v_x,v_y,v_z] and covariance
x_posterior_current = normrnd(2,0.1,[1,6])';
P_posterior = 1*eye(size(x_posterior_current,1));

scatter3(x_posterior_current(1),x_posterior_current(2),x_posterior_current(3),5,'r'); % plotting points
drawnow;

while index < iterations + 1
    
    % getting range measurements and time for single batch
    tic;
    z = getRangeMeasurement(serial)'/1000;
    dT = toc;
    
    [x_posterior_next,P_posterior] = VanillaEKF(anchor_pos,x_posterior_current,P_posterior,dT,z); % estimating a posteriori position
    
    scatter3(x_posterior_next(1),x_posterior_next(2),x_posterior_next(3),5,'r'); % plotting points
    
    % connecting neighboring positions
    line([x_posterior_current(1),x_posterior_next(1)], ...
        [x_posterior_current(2),x_posterior_next(2)], ...
        [x_posterior_current(3),x_posterior_next(3)],'Color',[1,.6196,.6196]);
    drawnow;
    
    % update 
    x_posterior_current = x_posterior_next;
    index = index + 1;
end




