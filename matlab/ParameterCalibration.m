% Calibrating anchor offsets and positions as well as tag positions
function ParameterCalibration = ParameterCalibration(ranges)

    % transform to [m] unit
    ranges = ranges/1000;
    
    % hardcoded pressure height
    pressure_height_tag = 0.225;
    pressure_height_bottom = 0.27;
    pressure_height_top = 2.43;
    
%% Parameters to estimate

    % 8 unknowns for range offsets
    syms o_0 o_1 o_2 o_3 o_4 o_5 o_6 o_7
    o_p = [o_0,o_1,o_2,o_3,o_4,o_5,o_6,o_7];

    % 12 unknowns for positions of antenna 4, 5 and 6   
    a_x = sym('a_x_',[1,3]);
    a_y = sym('a_y_',[1,3]);
    a_z = sym('a_z_',[1,3]);
    a_p = [a_x,a_y,a_z];

    % placement of the anchors
    room_width = 4;
    room_length = 4;
    anchor_height = pressure_height_top-pressure_height_bottom;

    anchor_pos = [0,room_width,0; ...
        room_length,room_width,anchor_height; ...
        room_length,0,0; ...
        0,0,anchor_height; ...
        a_x(1),a_y(1),a_z(1); ...
        a_x(2),a_y(2),a_z(2); ...
        a_x(3),a_y(3),a_z(3); ...
        0,0,0];
        
    % size(ranges, 2) * 3 unknowns
    p_x = sym('p_x_', [1,size(ranges, 2)]);
    p_y = sym('p_y_', [1,size(ranges, 2)]);
    p_z = sym('p_z_', [1,size(ranges, 2)]);
    p_p = [p_x,p_y,p_z];

%% Objective function to minimize
    
    index = 1; % for indexing the objective function
    f = sym(zeros(size(anchor_pos,1)*size(ranges,2),1));
    
    % ranging constraints
    for i = 1:size(ranges, 2)
        for j = 1:size(anchor_pos, 1)
            f(index) = sqrt((anchor_pos(j,1)-p_x(i))^2+(anchor_pos(j,2)-p_y(i))^2+ ...
                (anchor_pos(j,3)-p_z(i))^2)-ranges(j,i)-o_p(j);
            index = index+1;
        end
    end
    
    % height constraints for each measurement position
    for i = 1:size(ranges,2)
        f(index) = p_z(i)-(pressure_height_bottom-pressure_height_tag);
        index = index+1;
    end
    
    % height constraints for each anchor
    for j = 1:size(anchor_pos,1)
        if j == 1 || j == 3 || j == 6 || j == 8
            f(index) = anchor_pos(j,3)-(pressure_height_bottom-pressure_height_bottom);
            index = index+1;
        else
            f(index) = anchor_pos(j,3)-(pressure_height_top-pressure_height_bottom);
            index = index+1;
        end
    end
       
%% Gauss-Newton algorithm

    fp = jacobian(f,[o_p,p_p,a_p]); % calculate Jacobian   

    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);
    
    % allow array input
    f = convertToAcceptArray(f);
    fp = convertToAcceptArray(fp);

    % initial guess for offset
    o_i = normrnd(0.5, 0.1, [1, 8]);

    % initial guess for tag position
    p_x_i = normrnd(2,0.1,[1,size(ranges,2)]);
    p_y_i = normrnd(2,0.1,[1,size(ranges,2)]);
    p_z_i = normrnd(2,0.1,[1,size(ranges,2)]);

    % initial guess for anchor position
    a_x_i = normrnd(2,0.1,[1,3]);
    a_y_i = normrnd(2,0.1,[1,3]);
    a_z_i = normrnd(2,0.1,[1,3]);
        
    while true
        b = f([a_x_i,a_y_i,a_z_i,o_i,p_x_i,p_y_i,p_z_i]); % evaluate f

        A = fp([a_x_i,a_y_i,a_z_i,p_x_i,p_y_i,p_z_i]); % evaluate Jacobian

        d = - A\b; % solve linear least squares problem norm(A*d+b)=min

        % update
        opa = [o_i,p_x_i,p_y_i,p_z_i,a_x_i,a_y_i,a_z_i] + d';
        o_i = opa(1:8);
        p_x_i = opa(9:9+size(ranges,2)-1);
        p_y_i = opa(9+size(ranges,2):9+size(ranges,2)*2-1);
        p_z_i = opa(9+size(ranges,2)*2:9+size(ranges,2)*3-1);
        a_x_i = opa(9+size(ranges,2)*3:9+size(ranges,2)*3+3-1);
        a_y_i = opa(9+size(ranges,2)*3+3:9+size(ranges,2)*3+3*2-1);
        a_z_i = opa(9+size(ranges,2)*3+3*2:9+size(ranges,2)*3+3*3-1);

        if norm(d) <= 1e-10 % stop iteration of norm(d) <= StepTolerance
            break
        end
    end
     
     % show anchor offsets
     disp("Anchor Offsets:");
     disp(o_i);
     
%% Plotting

    anchor_pos = [0,room_width,0; ...
        room_length,room_width,anchor_height; ...
        room_length,0,0; ...
        0,0,anchor_height; ...
        a_x_i(1),a_y_i(1),a_z_i(1); ...
        a_x_i(2),a_y_i(2),a_z_i(2); ...
        a_x_i(3),a_y_i(3),a_z_i(3); ...
        0,0,0];
    
    figure()
    scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'o')
    hold on
    scatter3(p_x_i',p_y_i',p_z_i','o');    
end