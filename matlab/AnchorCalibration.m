% Johann Diep (jdiep@student.ethz.ch) - May 2019

% This program implements the method towards anchor position calibration
% described in the paper "Iterative approach for anchor configuration of
% positioning systems" by Mathias Pelka, Grigori Goronzy and Horst
% Hellbrueck.

function anchor_pos = AnchorCalibration(anchor_range_mean,plotting)
    %% Hardcoding values
    
    anchor_range_mean = anchor_range_mean/1000; % transform to [m] unit
    % anchors = 8; % for 8 anchor network
    anchors = 6; % for 6 anchors network
    height_top = 2.43; % anchors heights

    %% Preprocessing range data

    for i = 1:anchors
        for j = 1:anchors
            ranges_averaged(i,j) = (anchor_range_mean(i,j) + anchor_range_mean(j,i))/2;
        end
    end
            
    %% Parameters to estimate
     
    % placement of the anchors for 8 anchors network
    % anchor 1 is set to be the origin of the coordinate system
    % anchor 3, 6 and 8 are fixed on the same height as anchor 1
    % anchor 2, 4, 5 and 7 are fixed at a known constant height
    % anchor 6 is assumed to be on the same axis with anchor 1 without loss of generality
    % top anchors are assumed to have same x/y-coordinates as bottom anchors
    % syms a_3_x a_4_x
    % syms a_2_y a_3_y a_4_y

    % anchor_pos = [0,0,0; ...
        % 0,a_2_y,height_top; ...
        % a_3_x,a_3_y,0; ...
        % a_4_x,a_4_y,height_top; ...
        % 0,0,height_top; ...
        % 0,a_2_y,0; ...
        % a_3_x,a_3_y,height_top; ...
        % a_4_x,a_4_y,0];
        
    % a_p = [a_2_y,a_3_x,a_3_y,a_4_x,a_4_y];

    % placement of the anchors for 6 anchors network
    % anchor 1 is set to be the origin of the coordinate system
    % anchor 3 and 5 are fixed on the same height as anchor 1
    % anchor 2, 4 and 6 are fixed at a known constant height
    % anchor 5 is assumed to be on the same axis with anchor 1 without loss of generality
    % top anchors are assumed to have same x/y-coordinates as bottom anchors
    syms a_1_x
    syms a_1_y a_2_y

    anchor_pos = [0,0,0; ...
        0,0,height_top; ...
        a_1_x,a_1_y,0; ...
        a_1_x,a_1_y,height_top; ...
        0,a_2_y,0; ...
        0,a_2_y,height_top];

    a_p = [a_1_x,a_1_y,a_2_y];
    
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
    
    % a_i = normrnd(2,0.1,[1,size(a_p,2)]) % initialization with normal distribution
    % a_i = [2.0349,1.9271,2.0327,1.9485,1.9104]; % working initialization for 8 anchors network
    a_i = [1.8551,2.0334,2.0391]; % working initialization for 6 anchors network
        
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
    
    % for 8 anchors network
    % anchor_pos = [0,0,0; ...
    %     0,a_i(1),height_top; ...
    %     a_i(2),a_i(3),0; ...
    %     a_i(4),a_i(5),height_top; ...
    %     0,0,height_top; ...
    %     0,a_i(1),0; ...
    %     a_i(2),a_i(3),height_top; ...
    %     a_i(4),a_i(5),0];

    % for 6 anchors network
    anchor_pos = [0,0,0; ...
        0,0,height_top; ...
        a_i(1),a_i(2),0; ...
        a_i(1),a_i(2),height_top; ...
        0,a_i(3),0; ...
        0,a_i(3),height_top];

    if plotting == true
        figure()
        hold on
        title("Flying arena coordinate system");
        xlabel("x-Axis [m]");
        ylabel("y-Axis [m]");
        zlabel("z-Axis [m]");
        grid on

        scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'MarkerFaceColor',[0,0,0]);

        % for 8 anchors network
        % line([anchor_pos(1,1),anchor_pos(5,1)],[anchor_pos(1,2),anchor_pos(5,2)], ...
        %     [anchor_pos(1,3),anchor_pos(5,3)],'Color',[.5,.5,.5]);
        % line([anchor_pos(8,1),anchor_pos(4,1)],[anchor_pos(8,2),anchor_pos(4,2)], ...
        %     [anchor_pos(8,3),anchor_pos(4,3)],'Color',[.5,.5,.5]);
        % line([anchor_pos(6,1),anchor_pos(2,1)],[anchor_pos(6,2),anchor_pos(2,2)], ...
        %     [anchor_pos(6,3),anchor_pos(2,3)],'Color',[.5,.5,.5]);
        % line([anchor_pos(3,1),anchor_pos(7,1)],[anchor_pos(3,2),anchor_pos(7,2)], ...
        %     [anchor_pos(3,3),anchor_pos(7,3)],'Color',[.5,.5,.5]);

        % for 6 anchors network
        line([anchor_pos(1,1),anchor_pos(2,1)],[anchor_pos(1,2),anchor_pos(2,2)], ...
            [anchor_pos(1,3),anchor_pos(2,3)],'Color',[.5,.5,.5]);
        line([anchor_pos(3,1),anchor_pos(4,1)],[anchor_pos(3,2),anchor_pos(4,2)], ...
            [anchor_pos(3,3),anchor_pos(4,3)],'Color',[.5,.5,.5]);
        line([anchor_pos(5,1),anchor_pos(6,1)],[anchor_pos(5,2),anchor_pos(6,2)], ...
            [anchor_pos(5,3),anchor_pos(6,3)],'Color',[.5,.5,.5]);

        for i = 1:size(anchor_pos,1)
            text(anchor_pos(i,1)+0.1,anchor_pos(i,2)+0.1,anchor_pos(i,3)+0.1,"Anchor " + int2str(i));
        end
    end
end