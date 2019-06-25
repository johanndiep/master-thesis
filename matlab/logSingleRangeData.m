% Johann Diep (jdiep@student.ethz.ch) - June 2019
%
% This function logs the range measurements and their timestamps from the
% Bitcraze Positionin System setup as well as the ground-truth position and
% orientation from VICON over the ROS network. This function is needed for
% data analysis of orientation dependencies.
%
% Input:
%   - SerialObject: Serial port object
%   - ViconDroneSubscriber: ROS subscriber object to '/vicon/Bebop_Johann/Bebop_Johann' topic
%   - NumberOfIterations: Desired amount of batches of range measurement from all anchors
%
% Output:
%   - DronePositionGroundTruthArray: Stores the ground-truth positions in format [1, NumberOfIterations]
%   - DroneQuaternionGroundTruthArray: Stores the ground-truth quaternions in format [1, NumberOfIterations]
%   - RangeArray: Stores the range measurements in format [1, NumberOfIterations]
%   - TimeArray: Stores the times of the gathered ranges in format [1, NumberOfIterations]

function [DronePositionGroundTruthArray,DroneQuaternionGroundTruthArray,RangeArray,TimeArray] = logSingleRangeData(SerialObject,ViconDroneSubscriber,NumberOfIterations)
   IterationIndex = 1;
   FirstIteration = true;
   
   fopen(SerialObject); % run sudo chmod 666 /dev/ttyACM* on console first
   
   while IterationIndex < NumberOfIterations + 1
       LineSerial = fgetl(SerialObject);
       
       % starting readout with anchor 5 and avoid pre-information overload
       if FirstIteration == true
           while ~strncmpi(LineSerial,"Anchor 5",8)
               LineSerial = fgetl(SerialObject);
           end
       end
       
       if strncmpi(LineSerial,"Anchor 5",8)
          % starting the timer
          if FirstIteration == true
              TimeArray(IterationIndex) = 0;
              tic;
              FirstIteration = false;
          else
              TimeArray(IterationIndex) = toc;
          end
          
          RangeArray(IterationIndex) = str2double(LineSerial(24:end)); % storing range measurement in array
          [DronePositionGroundTruthArray(1:3,IterationIndex),DroneQuaternionGroundTruthArray(1:4,IterationIndex)] = getGroundTruth(ViconDroneSubscriber); % gather ground-truth data 
          
          IterationIndex = IterationIndex + 1; % update
       end
   end
   fclose(SerialObject);
end