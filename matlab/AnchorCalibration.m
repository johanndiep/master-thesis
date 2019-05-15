% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program implements the method towards anchor position calibration
% described in the paper "Iterative approach for anchor configuration of
% positioning systems" by Mathias Pelka, Grigori Goronzy and Horst
% Hellbrueck.

function AnchorCalibration = AnchorCalibration(range_mean)
    %% Hardcoding values
    
    range_mean = range_mean/1000; % transform to [m] unit
    
    anchors = 8; % number of anchors
    
    %anchor heights
    height_top = 2.43;

    %% Preprocessing range data

    for i = 1:anchors
        for j = 1:anchors
            ranges_averaged(i,j) = (range_mean(i,j) + range_mean(j,i))/2;
        end
    end
    
    ranges_averaged
        
    %% Parameters to estimate
     
    % placement of the anchors
    % anchor 1 is set to be the origin of the coordinate system
    % anchor 3, 6 and 8 are fixed on the same height as anchor 1
    % anchor 2, 4, 5 and 7 are fixed at a known constant height
    syms a_2_x a_3_x a_4_x a_6_x a_7_x a_8_x
    syms a_2_y a_3_y a_4_y a_6_y a_7_y a_8_y
    anchor_pos = [0,0,0; ...
        a_2_x,a_2_y,height_top; ...
        a_3_x,a_3_y,0; ...
        a_4_x,a_4_y,height_top; ...
        0,0,height_top; ...
        a_6_x,a_6_y,0; ...
        a_7_x,a_7_y,height_top; ...
        a_8_x,a_8_y,0];
    
end