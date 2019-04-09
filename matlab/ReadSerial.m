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
first_iteration = true;
averaging_number = 20;
range_offset = 0; % 0.557715 * 1000 for previous calibration value
next_column = false;

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

while index < iterations + 1
    if serial.Status == "closed"
        fopen(serial);
    end
    
    line = fgetl(serial);

    % start with anchor 0
    if first_iteration == true
        while ~strncmpi(line, "distance 0", 10)
            line = fgetl(serial);
        end

        first_iteration = false;
    end

        % storing range measurement in array of size (#anchors, #measurements) 
    if strncmpi(line, "distance", 8)    
        switch line(10)      
            case "0"
                range_array(1,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(1,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(1,index) = str2num(line(11:18));
                next_column = false;
            case "1"
                range_array(2,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(2,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(2,index) = str2num(line(11:18));
                next_column = false;
            case "2"
                range_array(3,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(3,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(3,index) = str2num(line(11:18));
                next_column = false;
            case "3"
                range_array(4,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(4,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(4,index) = str2num(line(11:18));
                next_column = false;
            case "4"
                range_array(5,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(5,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(5,index) = str2num(line(11:18));
                next_column = false;
            case "5"
                range_array(6,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(6,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(6,index) = str2num(line(11:18));
                next_column = false;
            case "6"
                range_array(7,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(7,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(7,index) = str2num(line(11:18));
                next_column = false;
            case "7"
                range_array(8,index) = str2num(line(13:17)) + range_offset;
                line = fgetl(serial);
                line = fgetl(serial);
                tag_pressure(8,index) = str2num(line(6:15));
                line = fgetl(serial);
                anchor_pressure(8,index) = str2num(line(11:18));
                next_column = true;
        end    
    end

    if next_column
        next_column = false;
        index = index + 1;

        if mod(size(range_array, 2), averaging_number) == 0
            fclose(serial); % close serial communication for tag displacement
            
            for a = 1:8
                range_mean(a, mean_index) = mean(rmoutliers(range_array(a, end-(averaging_number-1):end)), 2);
            end
            
            tag_pressure_inter = tag_pressure(:,end-(averaging_number-1):end);
            k = find(tag_pressure_inter);
            tag_pressure_mean(mean_index) = mean(mean(tag_pressure_inter(k)));
            
            if index ~= iterations + 1
               mean_index = mean_index +1;
               input("Change tag position to next calibration point.")
            end
        end
    end   
end

