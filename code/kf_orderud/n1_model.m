function result = n1_model (mode, x, u, n)
global p1 p2;

p1 = 4e-2;
p2 = 0.5;

Q = 1;
R = 1e-5;          

B = 0;          % input

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
global p1 p2;
F = p2;


function H = h_function (x, u, n)
% measurement jacobian
global p1 p2;
H = 2*p2*x;


function y = state_propagation (x, u, n)
% (linear) state transition
global p1 p2;
y  = 1 + p2*x + n;


function z = state_measure (x, u, n)
% perform measurement
global p1 p2;
z = p2*x.^2 + n;

