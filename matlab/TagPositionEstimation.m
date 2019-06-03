% Johann Diep (jdiep@student.ethz.ch) - May 2019

% Estimate the position of the tag with a set of range measurements with
% each anchor.

function tag_position = TagPositionEstimation(anchor_pos,range_array)
    %% Hardcoding values
    
    range_array = range_array/1000; % transform to [m] unit 
    anchors = 8; % number of anchors
    
    %% Parameters to estimate
    
    % coordinates of the current position of the tag
    syms p_x p_y p_z 
    p = [p_x,p_y,p_z];
    
    %% Objective function to minimize
    
    index = 1; 
    f = sym(zeros(anchors,1));
    
    %ranging constraints
    for i = 1:anchors
        f(index) = sqrt((p(1)-anchor_pos(index,1))^2 + ...
            (p(2)-anchor_pos(index,2))^2 + ...
            (p(3)-anchor_pos(index,3))^2) - range_array(index);
        index = index + 1;
    end
    
    %% Gauss-Newton algorithm
    
    fp = jacobian(f,p); % calculate Jacobian
    
    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);
    
    % allow function to handle array input
    f = convertToAcceptArray(f);
    fp = convertToAcceptArray(fp);
    
    % initialization
    p_i = normrnd(0,0.1,[1,size(p,2)]);
        
    while true
        b = f(p_i); % evaluate f
        A = fp(p_i); % evaluate Jacobian
        d = -A\b; % solve linear least squares problem
        
        p_i = p_i + d'; % update
        
        if norm(d) <= 1e-10 % stop iteration of norm(d) passes a tolerance
            break
        end
    end

    tag_position = p_i; % return solved position
end