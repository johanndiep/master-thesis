% Johann Diep (jdiep@student.ethz.ch) - August 2019
%
% This class contains methods which convert the gathered data from the
% Ultra-Wideband system into the desired format for Gaussian Process analysis.
% This is especially useful for exporting data to Python for parameter
% learning.

classdef DataPrep < handle
    properties
        RangeArray
    end
    
    methods
        % Initializing the data preprocessing object with the UWB range 
        % measurements between tag and anchor(s).
        %   - RangeArray: Range measurements in form (m x n)
        function DataPrepObj = DataPrep(RangeArray)
            DataPrepObj.RangeArray = RangeArray/1000;
        end
        
        % Returns the dataset for the tag-yaw-at-constant-distance
        % experiment. Hereby, the anchor stays at a fixed position.
        %   - DataPrepObj: Data preprocessing object defined by the constructor
        %   - ViconQuat: Vicon quaternion measurements of the tag in form (4 x n)
        %   - ActualDistance: Actual distance between the nodes in scalar form
        function [X,Y] = ConstDistanceYaw(DataPrepObj,ViconQuat,ActualDistance)
            RangeArray = DataPrepObj.RangeArray;
            
            ErrorArray = ActualDistance-RangeArray;
            Y = ErrorArray';
            
            X = quat2eul(ViconQuat');
            X(:,2:3) = [];
        end
        
        % Returns the dataset for the flight experiment.
        %   - DataPrepObj: Data preprocessing object defined by the constructor
        %   - ViconPos: Vicon position measurements of the drone in form (3 x n)
        function [X,Y] = Flight(DataPrepObj,ViconPos)
            RangeArray = DataPrepObj.RangeArray;
            
            Y = RangeArray;
            X = ViconPos;
        end
    end
end