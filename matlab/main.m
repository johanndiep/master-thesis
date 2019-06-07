% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This program is the main function which is used to call each executable.
% It will guide the user through each steps, starting from anchor setup
% self-calibration through gathering waypoint datas on the Bebop drone with
% the UWB-ranging method  as well as through the VICON system. 

clear;
clc;

disp("This program is the main function which is used to call each executable.");
disp("It will guide the user through each steps, starting from anchor setup");
disp("self-calibration through gathering waypoint datas on the Bebop drone with");
disp("the UWB-ranging method  as well as through the VICON system.");
disp("*************************************************");

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

%% Setup serial port

port = seriallist;
serial = serial(port);

%% Initialize ROS communication

rosinit;
rostopic_Bebop = '/vicon/Bebop_Johann/Bebop_Johann'; % listening to this topic for Bebop
rostopic_Anchors = '/vicon/Anchors_Johann/Anchors_Johann'; % listening to this topic for anchors
ViconSub_pos = rossubscriber(rostopic_Bebop); % creating subscriber object for Bebop
ViconAnchorsSub_pos = rossubscriber(rostopic_Anchors);  % creating subscriber object for anchors network
pause(5); % time needed for initialization

%% Hardcoded parameters and coordinate transformations

iterations = 100; % gather 50 position data
index = 1;

% measured deviation of VICON-frame from World-frame
X_ViconToWorld = 0.255;
Y_ViconToWorld = 0.22;
Z_ViconToWorld = -0.225;

% homogeneous transformation matrix, assumed to have no rotation
T_ViconToWorld = [1,0,0,X_ViconToWorld; ...
    0,1,0,Y_ViconToWorld; ...
    0,0,1,Z_ViconToWorld;
    0,0,0,1];

% assume no rotation between VICON anchors-frame and VICON-frame
anchors_rotation_gt = [1,0,0;0,1,0;0,0,1];

% read from VICON system
tag_BodyFrame = [11.873/1000;16.7275/1000;77.5299/1000];
anchors_12_BodyFrame = [-1161.76/1000;-1685.63/1000;22.9084/1000];
anchors_34_BodyFrame = [2351.09/1000;82.2879/1000;-33.8361/1000];
anchors_56_BodyFrame = [-1189.34/1000;1603.34/1000;10.9277/1000];

height_top = 2.43-0.275; % anchors heights
anchor_tag_deviation = 0.03; % measured deviation of anchor markers to bottom anchor

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

% for 8 anchors network
% line([anchor_pos(1,1),anchor_pos(5,1)],[anchor_pos(1,2),anchor_pos(5,2)], ...
%     [anchor_pos(1,3),anchor_pos(5,3)],'Color',[.5,.5,.5]);
% line([anchor_pos(8,1),anchor_pos(4,1)],[anchor_pos(8,2),anchor_pos(4,2)], ...
%     [anchor_pos(8,3),anchor_pos(4,3)],'Color',[.5,.5,.5]);
% line([anchor_pos(6,1),anchor_pos(2,1)],[anchor_pos(6,2),anchor_pos(2,2)], ...
%     [anchor_pos(6,3),anchor_pos(2,3)],'Color',[.5,.5,.5]);
% line([anchor_pos(3,1),anchor_pos(7,1)],[anchor_pos(3,2),anchor_pos(7,2)], ...
%     [anchor_pos(3,3),anchor_pos(7,3)],'Color',[.5,.5,.5]);

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

% plotting ground-truth anchors network
[anchors_position_gt,~] = getAnchorsGroundTruth(ViconAnchorsSub_pos);
anchors_12_gt = T_ViconToWorld * [anchors_rotation_gt,anchors_position_gt;0,0,0,1] * [anchors_12_BodyFrame;1];
anchors_34_gt = T_ViconToWorld * [anchors_rotation_gt,anchors_position_gt;0,0,0,1] * [anchors_34_BodyFrame;1];
anchors_56_gt = T_ViconToWorld * [anchors_rotation_gt,anchors_position_gt;0,0,0,1] * [anchors_56_BodyFrame;1];
scatter3(anchors_12_gt(1),anchors_12_gt(2),anchors_12_gt(3) - anchor_tag_deviation,'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);
scatter3(anchors_34_gt(1),anchors_34_gt(2),anchors_34_gt(3) - anchor_tag_deviation,'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);
scatter3(anchors_56_gt(1),anchors_56_gt(2),anchors_56_gt(3) - anchor_tag_deviation,'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);
scatter3(anchors_12_gt(1),anchors_12_gt(2),anchors_12_gt(3) + (height_top - anchor_tag_deviation),'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);
scatter3(anchors_34_gt(1),anchors_34_gt(2),anchors_34_gt(3) + (height_top - anchor_tag_deviation),'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);
scatter3(anchors_56_gt(1),anchors_56_gt(2),anchors_56_gt(3) + (height_top - anchor_tag_deviation),'MarkerEdgeColor','y','MarkerFaceColor',[1 1 0]);


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

% plotting starting position
range_array = getRangeMeasurement(serial);
tag_position_current = TagPositionEstimation(anchor_pos,range_array);
scatter3(tag_position_current(1),tag_position_current(2),tag_position_current(3),5,'r');

% plotting starting ground-truth position
[drone_position_gt_current,drone_quaternion_gt_current] = getGroundTruth(ViconSub_pos);
drone_rotation_gt_current = quat2rotm(drone_quaternion_gt_current');
tag_position_gt_current = T_ViconToWorld * [drone_rotation_gt_current,drone_position_gt_current;0,0,0,1] * [tag_BodyFrame;1];
scatter3(tag_position_gt_current(1),tag_position_gt_current(2),tag_position_gt_current(3),5,'b');

while index < iterations + 1
    disp("Position number " + index + " of " + iterations);
    
    % estimating tag position
    tic;
    range_array = getRangeMeasurement(serial);
    tag_position_next = TagPositionEstimation(anchor_pos,range_array);
    time = toc;
    disp("- New position estimated in " + time + " seconds");
    disp("- UWB position estimation frequency: " + 1/time + " Hz");
    
    % reading ground-truth position
    tic;
    [drone_position_gt,drone_quaternion_gt] = getGroundTruth(ViconSub_pos);
    drone_rotation_gt = quat2rotm(drone_quaternion_gt');
    tag_position_gt_next = T_ViconToWorld * [drone_rotation_gt,drone_position_gt;0,0,0,1] * [tag_BodyFrame;1];
    time = toc;
    disp("- New ground-truth position aquired in " + time + " seconds");
    disp("- VICON position estimation frequency: " + 1/time + " Hz");
    disp("*************************************************");
    
    % plotting the estimated and ground-truth positions
    scatter3(tag_position_next(1),tag_position_next(2),tag_position_next(3),5,'r');
    scatter3(tag_position_gt_next(1),tag_position_gt_next(2),tag_position_gt_next(3),5,'b');
    
    % connecting neighboring positions
    line([tag_position_current(1),tag_position_next(1)], ...
        [tag_position_current(2),tag_position_next(2)], ...
        [tag_position_current(3),tag_position_next(3)],'Color',[1,.6196,.6196]);
    line([tag_position_gt_current(1),tag_position_gt_next(1)], ...
        [tag_position_gt_current(2),tag_position_gt_next(2)], ...
        [tag_position_gt_current(3),tag_position_gt_next(3)],'Color',[.6196,.6196,1]);
    drawnow
    
    % update
    tag_position_current = tag_position_next;
    tag_position_gt_current = tag_position_gt_next;
    index = index + 1;
end

%% Closing ROS communication

disp("Finished gathering waypoints, shutting the system down");

rosshutdown;
