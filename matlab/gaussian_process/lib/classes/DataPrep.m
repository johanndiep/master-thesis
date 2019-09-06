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
        %   - VicQuat: Vicon quaternion measurements of the tag in form (4 x n)
        %   - ActualDistance: Actual distance between the nodes in scalar form
        function [X,Y] = ConstDistanceYaw(DataPrepObj,VicQuat,ActualDistance)
            RangeArray = DataPrepObj.RangeArray;
            
            ErrorArray = ActualDistance-RangeArray;
            Y = ErrorArray';
            
            X = quat2eul(VicQuat');
            X(:,2:3) = [];
        end
        
        % Returns the offsets for the UWB range measurement during flight
        % as well as the anchor positions in VICON frame and the ground-truth
        % ranges.
        %   - DataPrepObj: Data preprocessing object defined by the constructor
        %   - Marker: Sruct containing the marker positions in corresponding 
        %     body-frame 
        %   - VicDrPos: Vicon position measurements of the drone in form (3 x n)
        %   - VicDrQuat: Vicon quaternion measurements of the drone in form (4 x n)
        %   - VicAncPos: Vicon position measurement of the anchor system in
        %     form (3 x 1)
        %   - VicAncQuat: Vicon quaternion measurement of the anchor system
        %     in form (4 x 1)
        function [X,Y,A,P] = Flight(DataPrepObj,Marker,VicDrPos,VicDrQuat,VicAncPos,VicAncQuat)
            RangeArray = DataPrepObj.RangeArray;
            
            Dev = Marker.Dev;
            MarkP1 = Marker.MarkP1;
            MarkP2 = Marker.MarkP2;
            MarkP3 = Marker.MarkP3;
            MarkTag = Marker.MarkTag;
            
            Ta = diag(ones(1,4));
            Ta(1:3,1:3) = quat2rotm(VicAncQuat');
            Ta(1:3,4) = VicAncPos;
            
            A(:,1) = Ta*[MarkP1'+[0;0;Dev(1)];1];
            A(:,2) = Ta*[MarkP1'+[0;0;Dev(2)];1];
            A(:,3) = Ta*[MarkP2'+[0;0;Dev(1)];1];
            A(:,4) = Ta*[MarkP2'+[0;0;Dev(2)];1];
            A(:,5) = Ta*[MarkP3'+[0;0;Dev(1)];1];
            A(:,6) = Ta*[MarkP3'+[0;0;Dev(2)];1];
            A(4,:) = [];
            
            for i = 1:size(VicDrPos,2)
                Td = diag(ones(1,4));
                Td(1:3,1:3) = quat2rotm(VicDrQuat(:,i)');
                Td(1:3,4) = VicDrPos(:,i);
                
                X(:,i) = Td*[MarkTag';1];
            end
            X(4,:) = [];
            
            for j = 1:6
               B = repmat(A(:,j),1,size(X,2));
               P(j,:) = vecnorm(B-X);
               Y(j,:) = P(j,:)-RangeArray(j,:);
            end
        end
    end
end