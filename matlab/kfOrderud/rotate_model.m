function result = rotate_model (mode, x, u, n)
global k;

Q = [0.001 0;     % state noise covariance
     0      0.001];
R = 1.7;           % measurement covariance
B = [0; 0];
k = 0.1;

if     mode == 'F'  result = f_function(x,u,n);
elseif mode == 'H'  result = h_function(x,u,n);
elseif mode == 'B'  result = B;
elseif mode == 'Q'  result = Q;
elseif mode == 'R'  result = R;
elseif mode == 'S'  result = state_propagation(x,u,n);
elseif mode == 'M'  result = state_measure(x,u,n);
end

function F = f_function (x, u, n)
global k;
% state transition jacobian
px = x(1); py = x(2);

x_x = -sin(atan2(py, px) + k) * 1/(1 + py*py/px/px) * (-py/px/px);
x_y = -sin(atan2(py, px) + k) * 1/(1 + py*py/px/px) * (1/px);
y_x =  cos(atan2(py, px) + k) * 1/(1 + py*py/px/px) * (-py/px/px);
y_y =  cos(atan2(py, px) + k) * 1/(1 + py*py/px/px) * (1/px);

F = [x_x x_y;
     y_x y_y];    %discretize sate transition model


function H = h_function (x, u, n)
% measurement jacobian
H = [1 0];         


function y = state_propagation (x, u, n)
global k;
% state transition
theta = atan( x(2)/ x(1) ) + k;
y = [cos(theta); sin(theta)] + n;


function z = state_measure (x, u, n)
% measurement
H = h_function (x, u, n);
z = H * x + n;

