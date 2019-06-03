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
scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'s');

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

while true
    % starting position estimation procedure
    range_array = getRangeMeasurement(serial);
    tag_position = TagPositionEstimation(anchor_pos, range_array);
    disp("new position aquired");
    
    % plotting the positions
    scatter3(tag_position(1),tag_position(2),tag_position(3),'s');
    drawnow
end

