% Johann Diep (jdiep@student.ethz.ch) - April 2019

% This function calibrates the range measurement offset via Gauss-Newton. Thereby,
% it requires the anchor positions to be exactly measured. A anchor
% self-calibration method will follow soon.

function ParameterCalibration = ParameterCalibration(ranges)
    %% Hardcoding values
    
    ranges = ranges/1000; % transform to [m] unit
    
    % hardcoded pressure height
    pressure_height_tag = 0.225;
    pressure_height_bottom = 0.27;
    pressure_height_top = 2.43;
    
    d_coeff = 0.005; % damp value for Gauss-Newton update step
    
    %% Parameters to estimate (range offsets and tag locations, perhaps anchor positions)

    % 8 unknowns for range offsets
    syms o_0 o_1 o_2 o_3 o_4 o_5 o_6 o_7
    o_p = [o_0,o_1,o_2,o_3,o_4,o_5,o_6,o_7];

    % unknowns for positions of antennas, uncomment this to calibrate
    % anchor positions
%     a_x = sym('a_x_',[1,1]);
%     a_y = sym('a_y_',[1,1]);
%     a_z = sym('a_z_',[1,1]);
%     a_p = [a_x,a_y];
%     unknown_value = size(a_x,2);
    
    % placement of the anchors
    room_width = 4;
    room_length = 4;
    anchor_height = pressure_height_top-pressure_height_bottom;
    anchor_pos = [0,room_width,0; ...
        room_length,room_width,anchor_height; ...
        room_length,0,0; ...
        0,0,anchor_height; ...
        0,room_width,anchor_height; ...
        room_length,room_width,0; ...
        room_length,0,anchor_height; ...
        0,0,0];
        
    % locations, size(ranges, 2) * 3 unknowns
    p_x = sym('p_x_',[1,size(ranges,2)]);
    p_y = sym('p_y_',[1,size(ranges,2)]);
    p_z = sym('p_z_',[1,size(ranges,2)]);
    p_p = [p_x,p_y,p_z];

%% Objective function to minimize
    
    index = 1;
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
        f(index) = p_z(i)-(pressure_height_tag-pressure_height_bottom);
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

    fp = jacobian(f,[o_p,p_p]); % calculate Jacobian   

    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);
           
    % allow function to handle array input
    f = convertToAcceptArray(f);
    fp = convertToAcceptArray(fp);

    % initial guess for offset
    o_i = normrnd(0.5,0.1,[1,8]);

    % initial guess for tag position
    p_x_i = normrnd(2,0.1,[1,size(ranges,2)]);
    p_y_i = normrnd(2,0.1,[1,size(ranges,2)]);
    p_z_i = normrnd(2,0.1,[1,size(ranges,2)]);

    % initial guess for anchor position
%     a_x_i = normrnd(2,0.1,[1,unknown_value]);
%     a_y_i = normrnd(2,0.1,[1,unknown_value]);
%     a_z_i = normrnd(2,0.1,[1,unknown_value]);
            
    while true
        b = f([o_i,p_x_i,p_y_i,p_z_i]); % evaluate f
        disp("Objective Norm: " + norm(b));

        A = fp([p_x_i,p_y_i,p_z_i]); % evaluate Jacobian
        d = - A\b; % solve linear least squares problem

        % update
        opa = [o_i,p_x_i,p_y_i,p_z_i] + d_coeff*d';
        o_i = opa(1:8);
        p_x_i = opa(9:9+size(ranges,2)-1);
        p_y_i = opa(9+size(ranges,2):9+size(ranges,2)*2-1);
        p_z_i = opa(9+size(ranges,2)*2:9+size(ranges,2)*3-1);
%         a_x_i = opa(9+size(ranges,2)*3:9+size(ranges,2)*3+unknown_value-1);
%         a_y_i = opa(9+size(ranges,2)*3+unknown_value:9+size(ranges,2)*3+unknown_value*2-1);
%         a_z_i = opa(9+size(ranges,2)*3+unknown_value*2:9+size(ranges,2)*3+unknown_value*3-1);

        if norm(d) <= 1e-10 % stop iteration if norm(d) passes a tolerance
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
        0,room_width,anchor_height; ...
        room_length,room_width,0; ...
        room_length,0,anchor_height; ...
        0,0,0];
    
    figure()
    scatter3(anchor_pos(:,1),anchor_pos(:,2),anchor_pos(:,3),'o')
    hold on
    scatter3(p_x_i',p_y_i',p_z_i','o');    
end