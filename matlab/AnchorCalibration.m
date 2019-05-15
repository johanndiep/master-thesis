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
    distance_to_neighbor = 3.5; % manual measured distance from anchor 1 to 2

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
    syms a_2_x a_3_x a_4_x a_6_x a_7_x a_8_x
    syms a_2_y a_3_y a_4_y a_6_y a_7_y a_8_y
    anchor_pos = [0,0,0; ...
        0,distance_to_neighbor,height_top; ...
        a_3_x,a_3_y,0; ...
        a_4_x,a_4_y,height_top; ...
        0,0,height_top; ...
        0,distance_to_neighbor,0; ...
        a_7_x,a_7_y,height_top; ...
        a_8_x,a_8_y,0];
    a_p = [a_3_x,a_3_y,a_4_x,a_4_y,a_7_x,a_7_y,a_8_x,a_8_y];
    
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
    
    a_i = normrnd(2,0.1,[1,size(a_p,2)]); % initialization
        
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
        0,distance_to_neighbor,height_top; ...
        a_i(1),a_i(2),0; ...
        a_i(3),a_i(4),height_top; ...
        0,0,height_top; ...
        0,distance_to_neighbor,0; ...
        a_i(5),a_i(6),height_top; ...
        a_i(7),a_i(8),0];
    
    figure()
    scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'o')
        
end