function calibration_parameter = calibration_parameter(ranges)

% transform to [m] unit
    ranges = ranges / 1000;
    
%% Parameters

    syms o_0 o_1 o_2 o_3 o_4 o_5 o_6 o_7 % 8 unknowns for range offsets
    o_p = [o_0, o_1, o_2, o_3, o_4, o_5, o_6, o_7];

    % 12 unknowns for positions of antenna 1, 4, 5 and 6   
    a_x = sym('a_x_', [1, 3]);
    a_y = sym('a_y_', [1, 3]);
    a_z = sym('a_z_', [1, 3]);
    a_p = [a_x, a_y, a_z];

    % placement of the anchors
    room_width = 4;
    room_length = 4;
    anchor_height = 2.4;

    anchor_pos = [0, room_width, 0; ...
        room_length, room_width, anchor_height; ...
        room_length, 0, 0; ...
        0, 0, anchor_height; ...
        a_x(1), a_y(1), a_z(1); ...
        a_x(2), a_y(2), a_z(2); ...
        a_x(3), a_y(3), a_z(3); ...
        0, 0, 0];
        
    % size(ranges, 2) * 3 unknowns
    p_x = sym('p_x_', [1, size(ranges, 2)]);
    p_y = sym('p_y_', [1, size(ranges, 2)]);
    p_z = sym('p_z_', [1, size(ranges, 2)]);
 
    p_p = [p_x, p_y, p_z];

%% Objective function to minimize
    
    index = 1;
    f = sym(zeros(size(anchor_pos, 1) * size(ranges, 2),1));
    
    % ranging constraints
    for i = 1:size(ranges, 2)
        for j = 1:size(anchor_pos, 1)
            f(index) = sqrt((anchor_pos(j,1)-p_x(i))^2 + (anchor_pos(j,2)-p_y(i))^2 + ...
                (anchor_pos(j,3)-p_z(i))^2) - ranges(j,i) - o_p(j);
            index = index + 1;
        end
    end
    
    % hardcoding pressure height
    pressure_height_tag = 0.225;
    pressure_height_bottom = 0;
    pressure_height_top = 2.4;
    
    % height constraints for each measurement position
    for i = 1:size(ranges, 2)
        f(index) = p_z(i) - pressure_height_tag;
        index = index + 1;
    end
    
    % height constraints for each anchor
    for j = 1:size(anchor_pos, 1)
        if j == 1 || j == 3 || j == 6 || j == 8
            f(index) = anchor_pos(j,3) - pressure_height_bottom;
            index = index + 1;
        else
            f(index) = anchor_pos(j,3) - pressure_height_top;
            index = index + 1;
        end
    end
       
%% Gauss-Newton algorithm

    fp = jacobian(f, [o_p, p_p, a_p]); % calculate Jacobian   

    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);

    % initial guess for offset
    o_i = normrnd(0.5, 0.1, [1,8]);

    % initial guess for tag position
    p_x_i = normrnd(2, 0.1, [1, size(ranges, 2)]);
    p_y_i = normrnd(2, 0.1, [1, size(ranges, 2)]);
    p_z_i = normrnd(2, 0.1, [1, size(ranges, 2)]);

    % initial guess for anchor position
    a_x_i = normrnd(2, 0.1, [1,3]);
    a_y_i = normrnd(2, 0.1, [1,3]);
    a_z_i = normrnd(2, 0.1, [1,3]);
        
    while true
        % evaluate f
        b = f(a_x_i(1), a_x_i(2), a_x_i(3), ...
         a_y_i(1), a_y_i(2), a_y_i(3), ...
         a_z_i(1), a_z_i(2), a_z_i(3),...
         o_i(1),o_i(2),o_i(3),o_i(4),o_i(5),o_i(6),o_i(7),o_i(8), ...
         p_x_i(1),p_x_i(2),p_x_i(3),p_x_i(4),p_x_i(5),p_x_i(6),p_x_i(7),p_x_i(8),p_x_i(9), ...
         p_x_i(10),p_x_i(11),p_x_i(12),p_x_i(13),p_x_i(14),p_x_i(15),p_x_i(16),p_x_i(17),p_x_i(18), ...
         p_x_i(19), p_x_i(20), p_x_i(21),p_x_i(22),p_x_i(23),p_x_i(24),p_x_i(25),p_x_i(26), ...
         p_x_i(27),p_x_i(28),p_x_i(29), p_x_i(30), ...
         p_y_i(1),p_y_i(2),p_y_i(3),p_y_i(4),p_y_i(5),p_y_i(6),p_y_i(7),p_y_i(8),p_y_i(9), ...
         p_y_i(10),p_y_i(11),p_y_i(12),p_y_i(13),p_y_i(14),p_y_i(15),p_y_i(16),p_y_i(17),p_y_i(18), ...
         p_y_i(19), p_y_i(20), p_y_i(21),p_y_i(22),p_y_i(23),p_y_i(24),p_y_i(25),p_y_i(26), ...
         p_y_i(27),p_y_i(28),p_y_i(29), p_y_i(30), ...
         p_z_i(1),p_z_i(2),p_z_i(3),p_z_i(4),p_z_i(5),p_z_i(6),p_z_i(7),p_z_i(8),p_z_i(9), ...
         p_z_i(10),p_z_i(11),p_z_i(12),p_z_i(13),p_z_i(14),p_z_i(15),p_z_i(16),p_z_i(17),p_z_i(18), ...
         p_z_i(19),p_z_i(20), p_z_i(21),p_z_i(22),p_z_i(23),p_z_i(24),p_z_i(25),p_z_i(26), ...
         p_z_i(27),p_z_i(28),p_z_i(29), p_z_i(30));

        % evaluate Jacobian
        A = fp(a_x_i(1), a_x_i(2), a_x_i(3), ...
         a_y_i(1), a_y_i(2), a_y_i(3), ...
         a_z_i(1), a_z_i(2), a_z_i(3),...
         p_x_i(1),p_x_i(2),p_x_i(3),p_x_i(4),p_x_i(5),p_x_i(6),p_x_i(7),p_x_i(8),p_x_i(9), ...
         p_x_i(10),p_x_i(11),p_x_i(12),p_x_i(13),p_x_i(14),p_x_i(15),p_x_i(16),p_x_i(17),p_x_i(18), ...
         p_x_i(19), p_x_i(20), p_x_i(21),p_x_i(22),p_x_i(23),p_x_i(24),p_x_i(25),p_x_i(26), ...
         p_x_i(27),p_x_i(28),p_x_i(29), p_x_i(30), ...
         p_y_i(1),p_y_i(2),p_y_i(3),p_y_i(4),p_y_i(5),p_y_i(6),p_y_i(7),p_y_i(8),p_y_i(9), ...
         p_y_i(10),p_y_i(11),p_y_i(12),p_y_i(13),p_y_i(14),p_y_i(15),p_y_i(16),p_y_i(17),p_y_i(18), ...
         p_y_i(19), p_y_i(20), p_y_i(21),p_y_i(22),p_y_i(23),p_y_i(24),p_y_i(25),p_y_i(26), ...
         p_y_i(27),p_y_i(28),p_y_i(29), p_y_i(30), ...
         p_z_i(1),p_z_i(2),p_z_i(3),p_z_i(4),p_z_i(5),p_z_i(6),p_z_i(7),p_z_i(8),p_z_i(9), ...
         p_z_i(10),p_z_i(11),p_z_i(12),p_z_i(13),p_z_i(14),p_z_i(15),p_z_i(16),p_z_i(17),p_z_i(18), ...
         p_z_i(19),p_z_i(20), p_z_i(21),p_z_i(22),p_z_i(23),p_z_i(24),p_z_i(25),p_z_i(26), ...
         p_z_i(27),p_z_i(28),p_z_i(29), p_z_i(30));

        d = - A\b; % solve linear least squares problem norm(A*d+b)=min
         
        % update
        o_i = o_i + d(1:8)';
        p_x_i = p_x_i + d(9:38)';
        p_y_i = p_y_i + d(39:68)';
        p_z_i = p_z_i + d(69:98)';
        a_x_i = a_x_i + d(99:101)';
        a_y_i = a_y_i + d(102:104)';
        a_z_i = a_z_i + d(105:107)';
              
        if norm(d) <= 1e-10 % stop iteration of norm(d) <= StepTolerance
            break
        end
     end
     
     disp("Anchor Offsets:");
     disp(o_i);
     
%% Plotting

    anchor_pos = [0, room_width, 0; ...
        room_length, room_width, anchor_height; ...
        room_length, 0, 0; ...
        0, 0, anchor_height; ...
        a_x_i(1), a_y_i(1), a_z_i(1); ...
        a_x_i(2), a_y_i(2), a_z_i(2); ...
        a_x_i(3), a_y_i(3), a_z_i(3); ...
        0, 0, 0];
    
    figure()
    scatter3(anchor_pos(:,1), anchor_pos(:,2), anchor_pos(:,3), 'o')
    hold on
    scatter3(p_x_i', p_y_i',p_z_i', 'o');
    
    
end