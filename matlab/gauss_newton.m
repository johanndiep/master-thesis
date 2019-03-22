function p = gauss_newton(d)

% transform to [m] unit

d_0 = d(1)/1000;
d_1 = d(2)/1000;
d_2 = d(3)/1000;
d_3 = d(4)/1000;
d_4 = d(5)/1000;
d_5 = d(6)/1000;
d_6 = d(7)/1000;
d_7 = d(8)/1000;


%% Anchor positions

room_width = 4.045;
room_length = 4;
anchor_height_bottom = 0.27;
anchor_height_top = 2.43;

a_0 = [0, room_width, anchor_height_bottom];
a_1 = [room_length, room_width, anchor_height_top];
a_2 = [room_length, 0, anchor_height_bottom];
a_3 = [0, 0, anchor_height_top];
a_4 = [0, room_width, anchor_height_top];
a_5 = [room_length, room_width, anchor_height_bottom];
a_6 = [room_length, 0, anchor_height_bottom];
a_7 = [0, 0, anchor_height_bottom];

%% Objective function

f = @(p) [(a_0(1)-p(1))^2 + (a_0(2)-p(2))^2 + (a_0(3)-p(3))^2 - d_0^2; ...
    (a_1(1)-p(1))^2 + (a_1(2)-p(2))^2 + (a_1(3)-p(3))^2 - d_1^2; ...
    (a_2(1)-p(1))^2 + (a_2(2)-p(2))^2 + (a_2(3)-p(3))^2 - d_2^2; ...
    (a_3(1)-p(1))^2 + (a_3(2)-p(2))^2 + (a_3(3)-p(3))^2 - d_3^2; ...
    (a_4(1)-p(1))^2 + (a_4(2)-p(2))^2 + (a_4(3)-p(3))^2 - d_4^2; ...
    (a_5(1)-p(1))^2 + (a_5(2)-p(2))^2 + (a_5(3)-p(3))^2 - d_5^2; ...
    (a_6(1)-p(1))^2 + (a_6(2)-p(2))^2 + (a_6(3)-p(3))^2 - d_6^2; ...
    (a_7(1)-p(1))^2 + (a_7(2)-p(2))^2 + (a_7(3)-p(3))^2 - d_7^2];
  
%% Gauss-Newton algorithm

% Jacobian
fp = @(p) [-2*(a_0(1)-p(1)), -2*(a_0(2)-p(2)), -2*(a_0(3)-p(3)); ...
    -2*(a_1(1)-p(1)), -2*(a_1(2)-p(2)), -2*(a_1(3)-p(3)); ...
    -2*(a_2(1)-p(1)), -2*(a_2(2)-p(2)), -2*(a_2(3)-p(3)); ...
    -2*(a_3(1)-p(1)), -2*(a_3(2)-p(2)), -2*(a_3(3)-p(3)); ...
    -2*(a_4(1)-p(1)), -2*(a_4(2)-p(2)), -2*(a_4(3)-p(3)); ...
    -2*(a_5(1)-p(1)), -2*(a_5(2)-p(2)), -2*(a_5(3)-p(3)); ...
    -2*(a_6(1)-p(1)), -2*(a_6(2)-p(2)), -2*(a_6(3)-p(3)); ...
    -2*(a_7(1)-p(1)), -2*(a_7(2)-p(2)), -2*(a_7(3)-p(3))];

format long g % show all digits
p = [0; 0; 0]; % initial guess

while 1
  b = f(p); % evaluate f
  A = fp(p); % evaluate Jacobian
  
  d = -A\b; % solve linear least squares problem norm(A*d+b)=min
  p = p + d; % update
  
  if norm(d)<=1e-15 % stop iteration if norm(d)<=StepTolerance
    break
  end
end

%disp("p_x: " + p(1));
%disp("p_y: " + p(2));
%disp("p_z: " + p(3));

end
