function result = spring_model (mode, x, u, n)

Q = [0.0001 0;     % state noise covariance
     0      0.1];
R = 1.7;           % measurement covariance
B = [0; 0];

if     mode == 'F'  result = f_function(x,u,n);
elseif mode == 'H'  result = h_function(x,u,n);
elseif mode == 'B'  result = B;
elseif mode == 'Q'  result = Q;
elseif mode == 'R'  result = R;
elseif mode == 'S'  result = state_propagation(x,u,n);
elseif mode == 'M'  result = state_measure(x,u,n);
end

function F = f_function (x, u, n)
% state transition jacobian
k = 4; m = 1; dt = 0.1;   %parameters
Fc = [  0 1;        % continous state transition model
     -k/m 0];
F = expm(Fc*dt);    %discretize sate transition model


function H = h_function (x, u, n)
% measurement jacobian
%H = [1 0];         
H = [3*x(1)^2 0];         


function y = state_propagation (x, u, n)
% state transition
F = f_function(x, u, n);
y = F * x + n;


function z = state_measure (x, u, n)
% measurement
%H = h_function (x, u, n);
%z = H * x + n;
z = x(1)^3 + n;

