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
range_index = 1;
iterations = 2000;
first_iteration = true;
averaging_number = 100;
range_offset = 0; % 0.557715 * 1000; % previous calibration value

%% Range aquisition form each anchor via TWR

port = seriallist;
serial = serial(port);
fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first

while index < iterations + 1
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
                range_array(1, index) = str2num(line(13:17)) + range_offset;              
            case "1"
                range_array(2, index) = str2num(line(13:17)) + range_offset;               
            case "2"
                range_array(3, index) = str2num(line(13:17)) + range_offset;                 
            case "3"
                range_array(4, index) = str2num(line(13:17)) + range_offset;               
            case "4"
                range_array(5, index) = str2num(line(13:17)) + range_offset;               
            case "5"
                range_array(6, index) = str2num(line(13:17)) + range_offset;               
            case "6"
                range_array(7, index) = str2num(line(13:17)) + range_offset;               
            case "7"
                range_array(8, index) = str2num(line(13:17)) + range_offset;
        end    
    end  

    if strncmpi(line, "distance 7", 10)
        index = index + 1;
        
%         if mod(size(range_array, 2), averaging_number) == 0
%             range_mean = mean(range_array(:, end-(averaging_number-1):end), 2);
%             current_position = gauss_newton(range_mean);
%             disp(current_position);
%         end

        if mod(size(range_array, 2), averaging_number) == 0
            range_mean(:, range_index) = mean(range_array(:, end-(averaging_number-1):end), 2);
            
            if index ~= iterations + 1
                range_index = range_index +1;
                input("Change tag position to next calibration point.")
            end
        end
    end   
end

%calibration_parameter(range_mean/1000);

%% Histogram generation for visualization

% for j = 1:8
%     [range_outliers_removed, TF] = rmoutliers(range_array(j,:));
% 
%     subplot(2, 4, j);
%     histogram(range_outliers_removed, 50, 'FaceColor', 'y');
%     
%     title("Anchor " + j + ": " + mean(range_outliers_removed)/1000 + " m", 'FontWeight', 'normal');
% end