function calibration_parameter = calibration_parameter(ranges)

% transform to [m] unit
    ranges = ranges / 1000;

%% Parameters

    syms o_0 o_1 o_2 o_3 o_4 o_5 o_6 o_7 % 8 unknowns for range offsets
    
    % 12 unknowns for positions of antenna 1, 4, 5 and 6 
    syms a_x_1 a_y_1 a_z_1 
    syms a_x_4 a_y_4 a_z_4 
    syms a_x_5 a_y_5 a_z_5 
    syms a_x_6 a_y_6 a_z_6
    
    o_p = [o_0, o_1, o_2, o_3, o_4, o_5, o_6, o_7];
    
    a_x = sym('a_x_', [1, 4]);
    a_y = sym('a_y_', [1, 4]);
    a_z = sym('a_z_', [1, 4]);
    
    a_p = [a_x, a_y, a_z];

    % placement of the anchors
    room_width = 4;
    room_length = 4;
    anchor_height = 2.4;
    
%     anchor_pos = [0, room_width, 0; ...
%         room_length, room_width, anchor_height; ...
%         room_length, 0, 0; ...
%         0, 0, anchor_height; ...
%         0, room_width, anchor_height; ...
%         room_length, room_width, 0; ...
%         room_length, 0, anchor_height; ...
%         0, 0, 0];
    
    anchor_pos = [0, room_width, 0; ...
        a_x(1), a_y(1), a_z(1); ...
        room_length, 0, 0; ...
        0, 0, anchor_height; ...
        a_x(2), a_y(2), a_z(2); ...
        a_x(3), a_y(3), a_z(3); ...
        a_x(4), a_y(4), a_z(4); ...
        0, 0, 0];
        
    p_x = sym('p_x_', [1, size(ranges, 2)]);
    p_y = sym('p_y_', [1, size(ranges, 2)]);
    p_z = sym('p_z_', [1, size(ranges, 2)]);
 
    p_p = [p_x, p_y, p_z];

%% Objective function to minimize
    
    index = 1;
    f = sym(zeros(size(anchor_pos, 1) * size(ranges, 2),1));
    
    for i = 1:size(ranges, 2)
        for j = 1:size(anchor_pos, 1)
            f(index) = sqrt((anchor_pos(j,1)-p_x(i))^2 + (anchor_pos(j,2)-p_y(i))^2 + ...
                (anchor_pos(j,3)-p_z(i))^2) - ranges(j,i) - o_p(j);
            index = index + 1;
        end
    end

%% Gauss-Newton algorithm

    fp = jacobian(f, [o_p, p_p, a_p]); % calculate Jacobian
        
    % convert symbolic expression to function handle
    f = matlabFunction(f);
    fp = matlabFunction(fp);

    fp
    input("Wait")
    
    % initial guess for offset
    o_i = normrnd(0, 0.1, [1,8]);
    
    % initial guess for tag position
    p_x_i = normrnd(0, 0.1, [1, size(ranges, 2)]);
    p_y_i = normrnd(0, 0.1, [1, size(ranges, 2)]);
    p_z_i = normrnd(0, 0.1, [1, size(ranges, 2)]);
    
    % initial guess for anchor position
    a_x_i = normrnd(2, 0.1, [1, 4]);
    a_y_i = normrnd(2, 0.1, [1, 4]);
    a_z_i = normrnd(2, 0.1, [1, 4]);
    
     while true
         % evaluate f
         b = f(a_x_i(1), a_x_i(2), a_x_i(3), a_x_i(4), ...
             a_y_i(1), a_y_i(2), a_y_i(3), a_y_i(4), ...
             a_z_i(1), a_z_i(2), a_z_i(3), a_z_i(4), ...
             o_i(1),o_i(2),o_i(3),o_i(4),o_i(5),o_i(6),o_i(7),o_i(8), ...
             p_x_i(1),p_x_i(2),p_x_i(3),p_x_i(4),p_x_i(5),p_x_i(6),p_x_i(7),p_x_i(8),p_x_i(9), ...
             p_x_i(10),p_x_i(11),p_x_i(12),p_x_i(13),p_x_i(14),p_x_i(15),p_x_i(16),p_x_i(17),p_x_i(18), ...
             p_x_i(19), p_x_i(20), ...
             p_y_i(1),p_y_i(2),p_y_i(3),p_y_i(4),p_y_i(5),p_y_i(6),p_y_i(7),p_y_i(8),p_y_i(9), ...
             p_y_i(10),p_y_i(11),p_y_i(12),p_y_i(13),p_y_i(14),p_y_i(15),p_y_i(16),p_y_i(17),p_y_i(18), ...
             p_y_i(19), p_y_i(20), ...
             p_z_i(1),p_z_i(2),p_z_i(3),p_z_i(4),p_z_i(5),p_z_i(6),p_z_i(7),p_z_i(8),p_z_i(9), ...
             p_z_i(10),p_z_i(11),p_z_i(12),p_z_i(13),p_z_i(14),p_z_i(15),p_z_i(16),p_z_i(17),p_z_i(18), ...
             p_z_i(19),p_z_i(20));
        
         % evaluate Jacobian
         A = fp(a_x_i(1), a_x_i(2), a_x_i(3), a_x_i(4), ...
             a_y_i(1), a_y_i(2), a_y_i(3), a_y_i(4), ...
             a_z_i(1), a_z_i(2), a_z_i(3), a_z_i(4), ...
             p_x_i(1),p_x_i(2),p_x_i(3),p_x_i(4),p_x_i(5),p_x_i(6),p_x_i(7),p_x_i(8),p_x_i(9), ...
             p_x_i(10),p_x_i(11),p_x_i(12),p_x_i(13),p_x_i(14),p_x_i(15),p_x_i(16),p_x_i(17),p_x_i(18), ...
             p_x_i(19), p_x_i(20), ...
             p_y_i(1),p_y_i(2),p_y_i(3),p_y_i(4),p_y_i(5),p_y_i(6),p_y_i(7),p_y_i(8),p_y_i(9), ...
             p_y_i(10),p_y_i(11),p_y_i(12),p_y_i(13),p_y_i(14),p_y_i(15),p_y_i(16),p_y_i(17),p_y_i(18), ...
             p_y_i(19), p_y_i(20), ...
             p_z_i(1),p_z_i(2),p_z_i(3),p_z_i(4),p_z_i(5),p_z_i(6),p_z_i(7),p_z_i(8),p_z_i(9), ...
             p_z_i(10),p_z_i(11),p_z_i(12),p_z_i(13),p_z_i(14),p_z_i(15),p_z_i(16),p_z_i(17),p_z_i(18), ...
             p_z_i(19),p_z_i(20));
         
         d = - A\b; % solve linear least squares problem norm(A*d+b)=min
         
        % update
        o_i = o_i + d(1:8)';
        p_x_i = p_x_i + d(9:28)';
        p_y_i = p_y_i + d(29:48)';
        p_z_i = p_z_i + d(49:68)';
        a_x_i = a_x_i + d(69:72)';
        a_y_i = a_y_i + d(73:76)';
        a_z_i = a_z_i + d(77:80)';
              
        if norm(d) <= 1e-10 % stop iteration of norm(d) <= StepTolerance
            break
        end
     end
     
     disp("Anchor Offsets:");
     disp(o_i);
     
%% Plotting

    figure()
    plot(anchor_pos(:,1), anchor_pos(:,2), 'o')
    axis([-1 5 -1 5]);
    hold on
    plot(p_x_i', p_y_i', 'o');
    
    
end