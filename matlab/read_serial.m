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
iterations = 1000;
first_iteration = true;

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

while index < iterations + 1
disp(index);
    
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
                range_array(1, index) = str2num(line(13:17));              
             case "1"
                range_array(2, index) = str2num(line(13:17));               
             case "2"
                range_array(3, index) = str2num(line(13:17));                 
             case "3"
                range_array(4, index) = str2num(line(13:17));               
             case "4"
                range_array(5, index) = str2num(line(13:17));               
             case "5"
                range_array(6, index) = str2num(line(13:17));               
             case "6"
                range_array(7, index) = str2num(line(13:17));               
             case "7"
                range_array(8, index) = str2num(line(13:17));
         end    
    end  
  
    if str2num(line(10)) == 7
        index = index + 1;   
    end  
end

%% Histogram generation for visualization

for j = 1:8
    [range_outliers_removed, TF] = rmoutliers(range_array(j,:));

    subplot(2, 4, j);
    histogram(range_outliers_removed, 50, 'FaceColor', 'y');
    
    title("Anchor " + j + ": " + mean(range_outliers_removed)/1000 + " m", 'FontWeight', 'normal');
end