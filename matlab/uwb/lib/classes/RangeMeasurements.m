% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This class stores all the methods regarding range data acquisition from the
% Bitcraze Loco Positioning System setup. In total, there are 6 anchor nodes
% mounted on wooden poles and 1 tag node mounted on the Bebop drone. For more
% information, check out https://wiki.bitcraze.io/doc:lps:index.

classdef RangeMeasurements < handle
    properties
        SerialObject
        NrIterCal
        NrAnchors
    end
    
    methods
        % Initializing the range measurement object with the corresponding parameters.
        %   - NrIterCal: Amount of ranges to be gathered before averaged
        %     for anchor setup determination
        function RangeMeasObj = RangeMeasurements(NrIterCal)
            clear serial;
            if isempty(instrfind) == 0, fclose(instrfind); delete(instrfind); end
            PortAddress = seriallist;
            
            RangeMeasObj.SerialObject = serial(PortAddress);
            RangeMeasObj.NrIterCal = NrIterCal;
            RangeMeasObj.NrAnchors = 6;
        end
        
        % This function reads the distances between the anchors themself. The module
        % which is connected to the computer should be changed into the modified
        % Sniffer mode. The sniffer sequentially turns each single anchor into a tag,
        % which then starts ranging with its neighboring anchors. With the resulting
        % distances from the anchor setup, determination of an estimate the individual
        % anchor positions can be obtained in a later step.
        %   - RangeMeasObj: Measurement object defined by the constructor
        %
        % Furthermore, the following points need to be investigated:
        %   - Can one be sure, that after re-interrogating in case of a wrong tag number,
        %     no wrong tag number will follow?
        %   - Any delays introduced due to long reading processing?
        function AnchorRangeMean = AnchorSelfRanging(RangeMeasObj)
            SerialObject = RangeMeasObj.SerialObject;
            NrIterCal = RangeMeasObj.NrIterCal;
            NrAnchors = RangeMeasObj.NrAnchors;
            
            fopen(SerialObject); % run sudo chmod 666 /dev/ttyACM* in terminal first
            
            Iterdex = 1;
            Nextdex = 0;
            FirstIter = 1;
            
            RangeArr = zeros(NrAnchors,NrAnchors,NrIterCal); % pre-allocation
            
            for i = 1:NrAnchors % change each anchor to a tag once
                fwrite(SerialObject,'c'); % sending Switch-to-Tag command
                disp("Switch-to-Tag command sent to Anchor "+i)
                
                while Iterdex < NrIterCal+1
                    Line = fgetl(SerialObject);
                    
                    % re-interrogating in case of wrong tag number
                    while Line(17) ~= int2str(i), Line = fgetl(SerialObject); end
                    
                    % start reading with anchor 1/2
                    if FirstIter == 1
                        if i == 1
                            while strncmpi(Line,"Anchor 2",8) == 0
                                Line = fgetl(SerialObject);
                            end
                        elseif i > 1
                            while strncmpi(Line,"Anchor 1",8) == 0
                                Line = fgetl(SerialObject);
                            end
                        end
                        FirstIter = 0;
                    end
                    
                    % storing range measurement in an array
                    if strncmpi(Line,"Anchor",6)
                        switch LinearSerial(8)
                            case "1", RangeArr(i,1,Iterdex) = str2double(Line(24:end));
                            case "2", RangeArr(i,2,Iterdex) = str2double(Line(24:end));
                            case "3", RangeArr(i,3,Iterdex) = str2double(Line(24:end));
                            case "4", RangeArr(i,4,Iterdex) = str2double(Line(24:end));
                            case "5", RangeArr(i,5,Iterdex) = str2double(Line(24:end));
                                if i == NrAnchors, Nextdex = 1; end
                            case "6", RangeArr(i,6,Iterdex) = str2double(Line(24:end));
                                Nextdex = 1;
                        end
                    end
                    
                    if Nextdex == 1, Iterdex = Iterdex+1; Nextdex = 0; end
                    disp("Progress [%]: "+Iterdex/NrIterCal*100)
                end
                
                Iterdex = 1;
                FirstIter = 1;
                
                fwrite(SerialObject,"y"); % sending Switch-to-Anchor command
                pause(3);
            end
            
            fclose(SerialObject);
            
            % remove outliers and averaging
            AnchorRangeMean = zeros(NrAnchors,NrAnchors); % pre-allocation
            for Row = 1:NrAnchors
                for Column = 1:NrAnchors
                    AnchorRangeMean(Row,Column) = mean(rmoutliers(permute( ...
                        RangeArr(Row,Column,:),[1,3,2])));
                end
            end
        end
        
        % This function reads the distances between the drone-mounted tag and
        % each pole-mounted anchor.
        %   - RangeMeasObj: Measurement object defined by the constructor
        %
        % Furthermore, the following points need to be investigated:
        %   - How often does zero range measurements occur? How about other
        %     kind of outliers?
        function RangeArr = TagAnchorRanging(RangeMeasObj)
            SerialObject = RangeMeasObj.SerialObject;
            NrAnchors = RangeMeasObj.NrAnchors;
            
            fopen(SerialObject); % run sudo chmod 666 /dev/ttyACM* in terminal first
            
            FirstIter = 1;
            
            RangeArr = zeros(NrAnchors,1); % pre-allocation
            
            while true
                Line = fgetl(SerialObject);
                
                % start reading with anchor 1
                if FirstIter == 1
                    while strncmpi(Line,"Anchor 1",8) == 0
                        Line = fgetl(SerialObject);
                    end
                    FirstIter = 0;
                end
                
                % storing range measurement in an array
                if strncmpi(Line,"Anchor",6)
                    switch LinearSerial(8)
                        case "1", RangeArr(1,1) = str2double(Line(24:end));
                        case "2", RangeArr(2,1) = str2double(Line(24:end));
                        case "3", RangeArr(3,1) = str2double(Line(24:end));
                        case "4", RangeArr(4,1) = str2double(Line(24:end));
                        case "5", RangeArr(5,1) = str2double(Line(24:end));
                        case "6", RangeArr(6,1) = str2double(Line(24:end));
                            break;
                    end
                end
            end
            
            fclose(SerialObject);
        end
    end
end