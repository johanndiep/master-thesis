% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This program reads and stores the measurement from the Bitcraze Loco
% Positioning System setup. Hereby, one node functions as a tag and several 
% distributed around the space function as anchors.
%
% Input:
%   - SerialObject: Serial port object
%   - NumberOfIterations: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup
%
% Output:
%   - RangeMean: Stores the averaged range measurements in format [NumberOfAnchors,1]

function RangeMean = getRangeMeasurement(SerialObject,NumberOfIterations,NumberOfAnchors)    
    IterationIndex  = 1;
    NextIndex = false;
    FirstIteration = true;
    
    fopen(serial); % run sudo chmod 666 /dev/ttyACM* on console first
        
    while IterationIndex < NumberOfIterations + 1
        LineSerial = fgetl(SerialObject);
       
        % start readout with anchor 1 and avoid pre-information overload
        if FirstIteration == true
            while ~strncmpi(LineSerial,"Anchor 1",8)
                LineSerial = fgetl(SerialObject);
            end
           FirstIteration = false;
        end
              
        % storing range measurement in array of size (#anchors, #measurements)
        if strncmpi(LineSerial,"Anchor",6)
            switch LineSerial(8)
                case "1" % anchor 1
                    RangeArray(1,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "2" % anchor 2
                    RangeArray(2,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "3" % anchor 3
                    RangeArray(3,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "4" % anchor 4
                    RangeArray(4,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "5" % anchor 5
                    RangeArray(5,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "6" % anchor 6
                    RangeArray(6,IterationIndex) = str2double(LineSerial(24:end));
                    % differentiate between 6 and 6 anchors network
                    if NumberOfAnchors == 6
                        NextIndex = true;
                    else
                        NextIndex = false;
                    end
                case "7" % anchor 7
                    RangeArray(7,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "8" % anchor 8
                    RangeArray(8,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = true;
            end
        end
       
        if NextIndex % gather measurement for all anchors before updating index
            IterationIndex = IterationIndex + 1;
            NextIndex = false;
        end
    end
        
    fclose(SerialObject); % close serial communication for tag displacement
    
    % remove measurement outliers and average all ranges per location
    for ArrayRow = 1:NumberOfAnchors
        RangeMean(ArrayRow) = mean(rmoutliers(RangeArray(ArrayRow,:)));
    end
end