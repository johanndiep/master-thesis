clear
clc

%% Closing and deleting ports

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);   
end

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

index = 1;
iterations = 1000;
first_iteration = true;

while 1   
line = fgetl(serial);

    % avoid unfavourable first line serial read
    if first_iteration == true
        while ~strncmpi(line, "distance", 8)
            line = fgetl(serial);
        end
        
        first_iteration = false;
    end

    % storing range measurement in array of size (#anchors, #measurements) 
    if strncmpi(line, "distance", 8)    
         switch line(10)      
             case "0"
                distance_holder(1, index) = str2num(line(13:17));              
             case "1"
                distance_holder(2, index) = str2num(line(13:17));               
             case "2"
                distance_holder(3, index) = str2num(line(13:17));                 
             case "3"
                distance_holder(4, index) = str2num(line(13:17));               
             case "4"
                distance_holder(5, index) = str2num(line(13:17));               
             case "5"
                distance_holder(6, index) = str2num(line(13:17));               
             case "6"
                distance_holder(7, index) = str2num(line(13:17));               
             case "7"
                distance_holder(8, index) = str2num(line(13:17));
         end    
    end  
  
    if str2num(line(10)) == 7
        index = index + 1;   
    end  
end