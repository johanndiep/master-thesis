% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program reads and controls the Sniffer module.

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

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

for i = 1:anchors
    fwrite(serial, "c");
    while index < iterations + 1
        line = fgetl(serial);
       
        if first_iteration == true
            while ~strncmpi(line,"Anchor 1",8)
                line = fgetl(serial);
            end
            first_iteration = false;
        end
        
        if strncmpi(line,"Anchor",6)
            switch line(8)
                case "1"
                    range_array(i,1,index) = str2num(line(20:24));
                case "2"
                    range_array(i,2,index) = str2num(line(20:24));
                case "3"
                    range_array(i,3,index) = str2num(line(20:24));
                case "4"
                    range_array(i,4,index) = str2num(line(20:24));
                case "5"
                    range_array(i,5,index) = str2num(line(20:24));
                case "6"
                    range_array(i,6,index) = str2num(line(20:24));
            end
        end        
        index = index + 1;
    end  
    index = 0;
    fwrite(serial, "y");
    
    first_iteration = true;
end