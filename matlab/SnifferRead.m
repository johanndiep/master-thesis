% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program reads the distances from a Bitcraze Loco Positioning System anchor setup. 
% The module which is connected to the computer should be changed into the modified Sniffer mode.
% The sniffer sequentially turns each single anchor into a tag, which then starts ranging with
% its neighboring anchors. With the resulting distances from the anchor setup, self-calibration
% of the setup can be obtained.

clear 
clc

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);   
end

%% Parameters

index = 1;
next_index = false;
first_iteration = true;
iterations = 1000; % number of range data per anchor
anchors = 8; % number of anchors

%% Setup serial port

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

%% Range aquisition form each anchor via TWR

for i = 1:anchors % each anchor becomes a tag once
    disp("Interrogating anchor " + i);
    fwrite(serial, "c"); % sending switch_to_tag command, beginning at anchor 1 and ending at anchor 6
    while index < iterations + 1
        line = fgetl(serial);
       
        % start readout with anchor 1 and avoid pre-information overload
        if first_iteration == true
            if i == 1 % no information on anchor 1 if first anchor is tag
                while ~strncmpi(line,"Anchor 2",8)
                    line = fgetl(serial);
                end
            elseif i > 1
                while ~strncmpi(line,"Anchor 1",8)
                    line = fgetl(serial);
                end
            end
            first_iteration = false;
        end
        
        % Abortion statement
        if line(17) ~= int2str(i)
            disp("Aborting due to detection of wrong sequence number.");
            break;
        end
        
        % storing range measurement in array of size (#anchors, #anchors, #measurements)
        if strncmpi(line,"Anchor",6)
            switch line(8)
                case "1" % anchor 1
                    range_array(i,1,index) = str2num(line(24:end));
                    next_index = false;
                case "2" % anchor 2
                    range_array(i,2,index) = str2num(line(24:end));
                    next_index = false;
                case "3" % anchor 3
                    range_array(i,3,index) = str2num(line(24:end));
                    next_index = false;
                case "4" % anchor 4
                    range_array(i,4,index) = str2num(line(24:end));
                    next_index = false;
                case "5" % anchor 5
                    range_array(i,5,index) = str2num(line(24:end));
                    next_index = false;
                case "6" % anchor 6
                    range_array(i,6,index) = str2num(line(24:end));
                    next_index = false;
                case "7" % anchor 7
                    range_array(i,7,index) = str2num(line(24:end));
                    % solves indexing issue at last anchor index
                    if i == 8
                        next_index = true;
                    elseif i < 8
                        next_index = false;
                    end
                case "8" % anchor 8
                    range_array(i,8,index) = str2num(line(24:end));
                    next_index = true;
            end
        end
        
        if next_index % gather measurement for anchor 1 to 8 before updating index
           index = index + 1;
           next_index = false;
        end
    end
    
    index = 1;
    
    fwrite(serial, "y"); % sending switch_to_anchor command
    pause(2); % pause the system to avoid signal overload
    
    first_iteration = true;
end

%% Averaging measurements

for row = 1:anchors
   for column = 1:anchors
      range_mean(row,column) = mean(rmoutliers(permute(range_array(row,column,:),[1,3,2]))); % remove outliers and averaging 
   end
end
