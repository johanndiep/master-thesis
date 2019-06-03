% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This program is the main function which is used to call each executable.

clear;
clc;

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Setup serial port

port = seriallist;
serial = serial(port);

%% Calling anchor calibration executables

input("Place anchors in the room and press [ENTER]");
input("Change the additional module into Sniffer mode and press [ENTER]");

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

scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'MarkerFaceColor',[0,0,0]);

line([anchor_pos(1,1),anchor_pos(5,1)],[anchor_pos(1,2),anchor_pos(5,2)],[anchor_pos(1,3),anchor_pos(5,3)],'Color',[.5,.5,.5]);
line([anchor_pos(8,1),anchor_pos(4,1)],[anchor_pos(8,2),anchor_pos(4,2)],[anchor_pos(8,3),anchor_pos(4,3)],'Color',[.5,.5,.5]);
line([anchor_pos(6,1),anchor_pos(2,1)],[anchor_pos(6,2),anchor_pos(2,2)],[anchor_pos(6,3),anchor_pos(2,3)],'Color',[.5,.5,.5]);
line([anchor_pos(3,1),anchor_pos(7,1)],[anchor_pos(3,2),anchor_pos(7,2)],[anchor_pos(3,3),anchor_pos(7,3)],'Color',[.5,.5,.5]);

for i = 1:size(anchor_pos,1)
    text(anchor_pos(i,1)+0.1,anchor_pos(i,2)+0.1,anchor_pos(i,3)+0.1,"Anchor " + int2str(i));
end

%% Calling the position estimation executables

input("Change the additional module into Tag mode and press [ENTER]");

% delete and re-initialize serial
fclose(instrfind);
delete(instrfind);
delete(serial);
clear serial;
port = seriallist;
serial = serial(port);

% plotting starting position
range_array = getRangeMeasurement(serial);
tag_position_current = TagPositionEstimation(anchor_pos,range_array);
scatter3(tag_position_current(1),tag_position_current(2),tag_position_current(3),5,'r');

while true
    % estimating tag position trajectory
    tic;
    range_array = getRangeMeasurement(serial);
    tag_position_next = TagPositionEstimation(anchor_pos,range_array);
    time = toc;
    disp("New position aquired in " + time + " seconds");
    disp("Frequency: " + 1/time);
    
    % plotting the positions
    scatter3(tag_position_next(1),tag_position_next(2),tag_position_next(3),5,'r');
    line([tag_position_current(1),tag_position_next(1)],[tag_position_current(2),tag_position_next(2)],[tag_position_current(3),tag_position_next(3)],'Color',[1,.2,.2]);
    drawnow
    
    tag_position_current = tag_position_next;
end

