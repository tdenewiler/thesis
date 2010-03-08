function result = tracking_model (mode, x, u, n)
global B;

 Q = [.0001 0    0  0;     % state noise covariance
      0    .0001 0  0;
      0    0    .5  0;
      0    0    0  .5];

R = [ 5000 0;          % measurement covariance
      0   .001];          

B = [0; 0; 0; 0];          % input

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

F = [1 0 1 0;
     0 1 0 1;
     0 0 1 0;
     0 0 0 1];


function H = h_function (x, u, n)
% measurement jacobian

% dist  = sqrt( px*px + py*py );
px = x(1); py = x(2);

d_x = px/sqrt( px*px + py*py );
d_y = py/sqrt( px*px + py*py );

% angle = atan(x(2)/x(1));
a_x = 1/(1 + py*py/(px*px)) * (-py/(px*px));
a_y = 1/(1 + py*py/(px*px)) * (  1/px     );

H = [d_x d_y 0 0;
     a_x a_y 0 0];


function y = state_propagation (x, u, n)
% (linear) state transition
global B;

beta = -0.59783 * exp(x(5));
D = -beta * exp(


y(1) = x(1) + h*x(3);
y(2) = x(2) + h*x(4);
y(3) = x(3) + h*(D * x(3) + G * x(1));
y(4) = x(4) + h*(D * x(4) + G * x(2));
y(5) = x(5);

y = y + h*n;

function z = state_measure (x, u, n)
% perform measurement
%H = h_function (x, u, n, dt);

px = x(1); py = x(2);
dist  = sqrt( px*px + py*py );
angle = atan2(py,px); %atan(py/px);

z = [dist; angle] + n;

