% Johann Diep (jdiep@student.ethz.ch) - June 2019

% This function logs the range measurements and their timestamps from the
% Bitcraze Positioning System setup as well as the ground-truth position
% from VICON.

function range_mean = logRangeMeasurement(serial)
    %% Parameters
    
    anchors = 6;
    index = 1;
    next_index = false;
    first_iteration = true;
    iterations = 1;
    
    %% Open serial port
    
    fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first
    
    %% Range aquisition from each anchor via TWR
    
    while index < iterations + 1
        
    end
end