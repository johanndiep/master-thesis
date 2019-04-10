% Johann Diep (jdiep@student.ethz.ch) - April 2019

% This program reads and stores the measurement from the Bitcraze Loco
% Positioning System setup. Hereby, one node functions as a tag (connected to the computer)
% and 8 nodes distributed around the space function as anchors. 

clear 
clc

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);   
end

%% Parameters

anchors = 8;
index = 1;
mean_index = 1;
iterations = 100;
averaging_number = 50;
first_iteration = true;
next_column = false;

%% Range and pressure aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

while index < iterations + 1
    if serial.Status == "closed"
        fopen(serial);
    end
    
    line = fgetl(serial);

    % start with anchor 0 and avoid information overload
    if first_iteration == true
        while ~strncmpi(line,"distance 0",10)
            line = fgetl(serial);
        end
        first_iteration = false;
    end

    % storing range measurement in array of size (#anchors, #measurements)
    % storing tag pressure measurement in an array of size (#anchors, #measurements)
    % storing anchor pressure measurement in an array of size (#anchors, #measurements)
    if strncmpi(line, "distance", 8)    
        switch line(10)      
            case "0" % anchor 0
                range_array(1,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(1,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(1,index) = str2num(line(11:18));
                next_column = false;
            case "1" % anchor 1
                range_array(2,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(2,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(2,index) = str2num(line(11:18));
                next_column = false;
            case "2" % anchor 2
                range_array(3,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(3,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(3,index) = str2num(line(11:18));
                next_column = false;
            case "3" % anchor 3
                range_array(4,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(4,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(4,index) = str2num(line(11:18));
                next_column = false;
            case "4" % anchor 4
                range_array(5,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(5,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(5,index) = str2num(line(11:18));
                next_column = false;
            case "5" % anchor 5
                range_array(6,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(6,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(6,index) = str2num(line(11:18));
                next_column = false;
            case "6" % anchor 6
                range_array(7,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(7,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(7,index) = str2num(line(11:18));
                next_column = false;
            case "7" % anchor 7
                range_array(8,index) = str2num(line(13:17));
                line = fgetl(serial); % skip one serial line
                line = fgetl(serial);
                tag_pressure(8,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(8,index) = str2num(line(11:18));
                next_column = true;
        end    
    end

%% Averaging measurements per calibration position and relocalization of the tag    
    
    if next_column % gathered measurement for anchor 0 to 7
        next_column = false;
        index = index + 1;

        if mod(size(range_array,2),averaging_number) == 0 % enough measurements per location
            fclose(serial); % close serial communication for tag displacement
            
            % remove measurement outliers and average all ranges per location 
            for a = 1:anchors
                range_mean(a,mean_index) = mean(rmoutliers(range_array(a,end-(averaging_number-1):end)),2);
            end
            
            % remove measurement outliers and average all pressures per location
            tag_pressure_inter = tag_pressure(:,end-(averaging_number-1):end);
            k = find(tag_pressure_inter);
            tag_pressure_mean(mean_index) = mean(mean(tag_pressure_inter(k)));
            
            if index ~= iterations + 1
               mean_index = mean_index + 1;
               input("Change tag position to next calibration point.")
               first_iteration = true;
            end
        end
    end    
end

% remove measurement outliers and average all pressures per anchor
[ii,~,v] = find(anchor_pressure);
anchor_pressure_mean = accumarray(ii,v,[],@mean);