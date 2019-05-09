% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program reads and controls the modified Sniffer module.

clear 
clc

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);   
end

%% Parameters

index = 1;
iterations = 100;
averaging_number = 50;
anchors = 6;
first_iteration = true;
next_index = false;

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

for i = 1:anchors
    fwrite(serial, "c");
    while index < iterations + 1
        line = fgetl(serial);
       
        % start with anchor 1 and avoid information overload
        if first_iteration == true
            if i == 1
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
                    
                    % solves indexing issue at last anchor index
                    if i == 6 
                        next_index = true;
                    elseif i < 6
                        next_index = false;
                    end
                case "6" % anchor 6
                    range_array(i,6,index) = str2num(line(24:end));
                    next_index = true;
            end
        end
        
        if next_index % gathered measurement for anchor 1 to 6
           index = index + 1;
           next_index = false;
        end
    end
    
    index = 1;
    fwrite(serial, "y");
    pause(2);
    first_iteration = true;
end

%% Averaging measurements

for row = 1:anchors
   for column = 1:anchors
      range_mean(row,column) = mean(rmoutliers(permute(range_array(row,column,:),[1,3,2]))); 
   end
end

