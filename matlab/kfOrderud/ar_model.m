function result = rotate_model (mode, x, u, n)
global k;

Q = [0.01 0    0;     % state noise covariance
     0      0.01 0;
     0      0     .01];
R = 0.1;           % measurement covariance
B = [0; 0; 0];
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
px = x(1); py = x(2); pz = x(3);

x_x = 0.9;
x_y =  0.2 /(1+ pz^2);
x_z = -0.4*pz*py/(1+pz^2)^2;

F = [x_x x_y x_z;
     1   0   0;    %discretize sate transition model
     0   1   0 ];

function H = h_function (x, u, n)
% measurement jacobian
H = [1 0 0];         


function y = state_propagation (x, u, n)
% state transition
px = x(1); py = x(2); pz = x(3);

f = 0.9*px + 0.2*py/(1 + pz^10);
y = [f; x(1); x(2)] + n;


function z = state_measure (x, u, n)
% measurement
H = h_function (x, u, n);
z = H * x + n;

