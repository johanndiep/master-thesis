% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function logs the range measurements and their timestamps from the
% Bitcraze Positioning System setup as well as the ground-truth position
% from VICON over the ROS network. This function can be used to gather data
% for parameter tuning as well as analyzing different localization
% approaches.
%
% Input:
%   - SerialObject: Serial port object
%   - ViconDroneSubscriber: ROS subscriber object to '/vicon/Bebop_Johann/Bebop_Johann' topic
%   - NumberOfIterations: Desired amount of batches of range measurement from all anchors
%   - NumberOfAnchors: Amount of anchors in the setup
%
% Output:
%   - TimeArray: Stores the times of the gathered ranges in format [NumberofAnchors, NumberofIterations]
%   - RangeArray: Stores the range measurements in format [NumberOfAnchors, NumberOfIterations]
%   - DronePositionGroundTruthArray: Stores the ground-truth positions 
%   - DroneQuaternionGroundTruthArray: Stores the ground-truth quaternions

function [TimeArray,RangeArray,DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray] = logRangeMeasurement(SerialObject,ViconDroneSubscriber,NumberOfIterations,NumberOfAnchors)    
    IterationIndex = 1;
    GroundTruthIterationIndex = 1;
    NextIndex = false;
    FirstIteration = true;
    
    fopen(SerialObject); % run sudo chmod 666 /dev/ttyACM* on console first
        
    while IterationIndex < NumberOfIterations + 1
        LineSerial = fgetl(SerialObject);
        
        % starting readout with anchor 1 and avoid pre-information overload
        if FirstIteration == true
            while ~strncmpi(LineSerial,"Anchor 1",8)
                LineSerial = fgetl(SerialObject); 
            end
        end
        
        [DronePositionGroundTruthArray(1:3,GroundTruthIterationIndex),DroneQuaternionGroundTruthArray(1:4,GroundTruthIterationIndex)] = getGroundTruth(ViconDroneSubscriber); % gather ground-truth data
        GroundTruthIterationIndex = GroundTruthIterationIndex + 1;
        
        % storing range measurement in array
        if strncmpi(LineSerial,"Anchor",6)
            switch LineSerial(8)
                case "1" % anchor 1
                    % starting the timer
                    if FirstIteration == true
                        TimeArray(1,IterationIndex) = 0;
                        tic;                        
                        FirstIteration = false;
                    else
                        TimeArray(1,IterationIndex) = toc;
                    end
                    RangeArray(1,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "2" % anchor 2
                    TimeArray(2,IterationIndex) = toc;
                    RangeArray(2,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "3" % anchor 3
                    TimeArray(3,IterationIndex) = toc;
                    RangeArray(3,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "4" % anchor 4
                    TimeArray(4,IterationIndex) = toc;
                    RangeArray(4,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "5" % anchor 5
                    TimeArray(5,IterationIndex) = toc;
                    RangeArray(5,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "6" % anchor 6
                    TimeArray(6,IterationIndex) = toc;
                    RangeArray(6,IterationIndex) = str2double(LineSerial(24:end));
                    % differentiate between 6 and 8 anchors network
                    if NumberOfAnchors == 6
                        NextIndex = true;
                    else
                        NextIndex = false;
                    end
                case "7" % anchor 7
                    TimeArray(7,IterationIndex) = toc;
                    RangeArray(7,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = false;
                case "8" % anchor 8
                    TimeArray(8,IterationIndex) = toc;
                    RangeArray(8,IterationIndex) = str2double(LineSerial(24:end));
                    NextIndex = true;
            end
        end
        
        if NextIndex % gather measurement for all anchors before updating index
            IterationIndex = IterationIndex + 1;
            NextIndex = false;
        end
    end
end