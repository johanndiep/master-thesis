% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This program reads and stores the measurement from the Bitcraze Loco
% Positioning System setup. Hereby, one node functions as a tag (fixed on the drone)
% and 8 nodes distributed around the space function as anchors.

function range_mean = getRangeMeasurement(serial)
    %% Parameters
    
    % anchors = 8; % for 8 anchors network
    anchors = 6; % for 6 anchors network
    index  = 1;
    next_index = false;
    first_iteration = true;
    iterations = 5;
    
    %% Open serial port

    fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first
    
    %% Range aquisition from each anchor via TWR
    
    while index < iterations + 1
       line = fgetl(serial);
       
       % start readout with anchor 1 and avoid pre-information overload
       if first_iteration == true
          while ~strncmpi(line,"Anchor 1",8)
              line = fgetl(serial);
          end
          first_iteration = false;
       end
              
       % storing range measurement in array of size (#anchors, #measurements)
       if strncmpi(line,"Anchor",6)
           switch line(8)
               case "1" % anchor 1
                   range_array(1,index) = str2double(line(24:end));
                   next_index = false;
               case "2" % anchor 2
                   range_array(2,index) = str2double(line(24:end));
                   next_index = false;
               case "3" % anchor 3
                   range_array(3,index) = str2double(line(24:end));
                   next_index = false;
               case "4" % anchor 4
                   range_array(4,index) = str2double(line(24:end));
                   next_index = false;
               case "5" % anchor 5
                   range_array(5,index) = str2double(line(24:end));
                   next_index = false;
               case "6" % anchor 6
                   range_array(6,index) = str2double(line(24:end));
                   % differentiate between 6 and 6 anchors network
                   if anchors == 6
                       next_index = true;
                   else
                       next_index = false;
                   end
               case "7" % anchor 7
                   range_array(7,index) = str2double(line(24:end));
                   next_index = false;
               case "8" % anchor 8
                   range_array(8,index) = str2double(line(24:end));
                   next_index = true;
           end
       end
       
       if next_index % gather measurement for all anchors before updating index
           index = index + 1;
           next_index = false;
       end
    end
    
    %% Averaging measurements per position
    
    fclose(serial); % close serial communication for tag displacement
    
    % remove measurement outliers and average all ranges per location
    for row = 1:anchors
        range_mean(row) = mean(rmoutliers(range_array(row,:)));
    end
end