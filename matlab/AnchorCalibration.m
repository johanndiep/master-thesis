% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program implements the method towards anchor position calibration
% described in the paper "Iterative approach for anchor configuration of
% positioning systems" by Mathias Pelka, Grigori Goronzy and Horst
% Hellbrueck.

function AnchorCalibration = AnchorCalibration(range_mean)
    %% Hardcoding values
    
    range_mean = range_mean/1000; % transform to [m] unit
    anchors = 8; % number of anchors
    height_top = 2.43; % anchor heights

    %% Preprocessing range data

    for i = 1:anchors
        for j = 1:anchors
            ranges_averaged(i,j) = (range_mean(i,j) + range_mean(j,i))/2;
        end
    end
            
    %% Parameters to estimate
     
    % placement of the anchors
    % anchor 1 is set to be the origin of the coordinate system
    % anchor 3, 6 and 8 are fixed on the same height as anchor 1
    % anchor 2, 4, 5 and 7 are fixed at a known constant height
    % anchor 6 is assumed to be on the same axis without loss of generality
    syms a_2_x a_3_x a_4_x a_5_x a_7_x a_8_x
    syms a_2_y a_3_y a_4_y a_5_y a_6_y a_7_y a_8_y
    anchor_pos = [0,0,0; ...
        a_2_x,a_2_y,height_top; ...
        a_3_x,a_3_y,0; ...
        a_4_x,a_4_y,height_top; ...
        a_5_x,a_5_y,height_top; ...
        0,a_6_y,0; ...
        a_7_x,a_7_y,height_top; ...
        a_8_x,a_8_y,0];
    a_p = [a_2_x,a_2_y,a_3_x,a_3_y,a_4_x,a_4_y,a_5_x,a_5_y,a_6_y,a_7_x,a_7_y,a_8_x,a_8_y];
    
    %% Objective function to minimize
    
    index = 1; 
    f = sym(zeros(28,1));
    
    %ranging constraints
    for i = 1:anchors
        for j = 1:anchors
            if j == i || isequal([i,j],permute([i,j],[1,2])') % avoid identity and equal comparisons
                break;
            end
            f(index) =  sqrt((anchor_pos(i,1)-anchor_pos(j,1))^2 + ...
                (anchor_pos(i,2)-anchor_pos(j,2))^2 + ...
                (anchor_pos(i,3)-anchor_pos(j,3))^2) - ranges_averaged(i,j);
            index = index + 1;
        end
    end
        
    %% Gauss-Newton algorithm
    
    fp = jacobian(f,a_p); % calculate Jacobian
    
    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);    
    
    % allow function to handle array input
    f = convertToAcceptArray(f);
    fp = convertToAcceptArray(fp);
    
    % initialization
    % a_i = normrnd(10,0.1,[1,size(a_p,2)]);
    a_i = [10.0553,10.1039,9.8882,10.1261,10.0660,9.9932,9.9805,9.9782,9.9697,10.0023,10.0051,10.0826,10.1527];
        
    while true
        b = f(a_i); % evaluate f
        A = fp(a_i); % evaluate Jacobian
        d = -A\b; % solve linear least squares problem
        
        % update
        opa = a_i + d';
        a_i = opa;
        
        if norm(d) <= 1e-10 % stop iteration if norm(d) passes a tolerance
            break
        end
    end
    
    %% Plotting
    
    anchor_pos = [0,0,0; ...
        a_i(1),a_i(2),height_top; ...
        a_i(3),a_i(4),0; ...
        a_i(5),a_i(6),height_top; ...
        a_i(7),a_i(8),height_top; ...
        0,a_i(9),0; ...
        a_i(10),a_i(11),height_top; ...
        a_i(12),a_i(13),0];
    
    figure()
    scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'s')
    
    for i = 1:size(anchor_pos,1)
        text(anchor_pos(i,1)+0.1,anchor_pos(i,2)+0.1,anchor_pos(i,3)+0.1,"Anchor " + int2str(i));
    end
end