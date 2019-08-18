% Johann Diep (jdiep@student.ethz.ch) - May 2019
%
% This program reads the distances from a Bitcraze Loco Positioning System 
% anchor setup. The module which is connected to the computer should be 
% changed into the modified Sniffer mode. The sniffer sequentially turns 
% each single anchor into a tag, which then starts ranging with its 
% neighboring anchors. With the resulting distances from the anchor setup, 
% self-calibration of the setup can be obtained.
%
% Input:
%   - SerialObject: Serial port object
%   - NumberOfIterationsForCalibration: Desired amount of ranges to be gathered before averaged
%   - NumberOfAnchors: Amount of anchors in the setup
% 
% Output:
%   - AnchorRangeMean: Stores the averaged ranges measurements between each anchor in format [NumberofAnchors, NumberofAnchors]

function AnchorRangeMean = getAnchorRangeMeasurement(SerialObject,NumberOfIterationsForCalibration,NumberOfAnchors)
    IterationIndex = 1;
    NextIndex = false;
    FirstIteration = true;

    fopen(SerialObject); % run sudo chmod 666 /dev/ttyACM* on console first

    for AnchorIterationIndex = 1:NumberOfAnchors % each anchor becomes a tag once
        fwrite(SerialObject, "c"); % sending Switch-to-Tag command, beginning at anchor 1 and ending at last anchor
        while IterationIndex < NumberOfIterationsForCalibration + 1
            LineSerial = fgetl(SerialObject);

            % start readout with anchor 1 and avoid pre-information overload
            if FirstIteration == true
                ProgressBar = waitbar(0,"Interrogating anchor " + AnchorIterationIndex); % creating waitbar     
                if AnchorIterationIndex == 1 % no information on anchor 1 if first anchor is tag
                    while ~strncmpi(LineSerial,"Anchor 2",8)
                        LineSerial = fgetl(SerialObject);
                    end
                elseif AnchorIterationIndex > 1
                    while ~strncmpi(LineSerial,"Anchor 1",8)
                        LineSerial = fgetl(SerialObject);
                    end
                end
                FirstIteration = false;
            end

            % in case of overlapping
            while LineSerial(17) ~= int2str(AnchorIterationIndex)
                disp("WARNING: Detection of wrong sequence number, re-interrogating ...");
                LineSerial = fgetl(SerialObject);
            end

            % storing range measurement in an array
            if strncmpi(LineSerial,"Anchor",6)
                switch LineSerial(8)
                    case "1" % anchor 1
                        RangeArray(AnchorIterationIndex,1,IterationIndex) = str2double(LineSerial(24:end));
                        NextIndex = false;
                    case "2" % anchor 2
                        RangeArray(AnchorIterationIndex,2,IterationIndex) = str2double(LineSerial(24:end));
                        NextIndex = false;
                    case "3" % anchor 3
                        RangeArray(AnchorIterationIndex,3,IterationIndex) = str2double(LineSerial(24:end));
                        NextIndex = false;
                    case "4" % anchor 4
                        RangeArray(AnchorIterationIndex,4,IterationIndex) = str2double(LineSerial(24:end));
                        NextIndex = false;
                    case "5" % anchor 5
                        RangeArray(AnchorIterationIndex,5,IterationIndex) = str2double(LineSerial(24:end));
                        % differentiate between 6 and 8 anchors network
                        if NumberOfAnchors == 6
                            % solves indexing issue at last anchor index
                            if AnchorIterationIndex == 6
                                NextIndex = true;
                            elseif AnchorIterationIndex < 6
                                NextIndex = false;
                            end
                        else
                            NextIndex = false;
                        end
                    case "6" % anchor 6
                        RangeArray(AnchorIterationIndex,6,IterationIndex) = str2double(LineSerial(24:end));
                        % differentiate between 6 and 8 anchors network
                        if NumberOfAnchors == 6
                            NextIndex = true;
                        else
                            NextIndex = false;
                        end
                    case "7" % anchor 7
                        RangeArray(AnchorIterationIndex,7,IterationIndex) = str2double(LineSerial(24:end));
                        % solves indexing issue at last anchor index
                        if AnchorIterationIndex == 8
                            NextIndex = true;
                        elseif AnchorIterationIndex < 8
                            NextIndex = false;
                        end
                    case "8" % anchor 8
                        RangeArray(AnchorIterationIndex,8,IterationIndex) = str2double(LineSerial(24:end));
                        NextIndex = true;
                end
            end

            if NextIndex % gather measurement for anchor 1 to last anchor before updating index
                IterationIndex = IterationIndex + 1;
                waitbar(IterationIndex/(NumberOfIterationsForCalibration + 1),ProgressBar,"Interrogating anchor " + AnchorIterationIndex);
                NextIndex = false;
            end
        end

        IterationIndex = 1;

        fwrite(SerialObject, "y"); % sending Switch-to-Anchor command
        pause(2); % pause the system to avoid signal overload
        
        close(ProgressBar);

        FirstIteration = true;
    end  
    
    fclose(SerialObject);

    % remove outliers and averaging
    for ArrayRow = 1:NumberOfAnchors
        for ArrayColumn = 1:NumberOfAnchors
           AnchorRangeMean(ArrayRow,ArrayColumn) = mean(rmoutliers(permute(RangeArray(ArrayRow,ArrayColumn,:),[1,3,2]))); 
        end
    end
end

