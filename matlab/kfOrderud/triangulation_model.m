function result = triangulation_model (mode, x, u, n)
global B p1 p2;

 Q = [.0001 0    0  0;     % state noise covariance
      0    .0001 0  0;
      0    0    .5  0;
      0    0    0  .5];

R = [ 200 0;          % measurement covariance
      0   200];          

B = [0; 0; 0; 0];          % input

% sensor locations
p1 = [-300 0];
p2 = [ 300 0];

if     mode == 'F'  result = f_function(x,u,n);
elseif mode == 'H'  result = h_function(x,u,n);
elseif mode == 'B'  result = B;

elseif mode == 'Q'  result = Q;
elseif mode == 'R'  result = R;

elseif mode == 'S'  result = state_propagation(x,u,n);
elseif mode == 'M'  result = state_measure(x,u,n);

elseif mode == 'tria'  result = triangulate_measurement(p1, p2, x, u);

end


function F = f_function (x, u, n)
% state transition jacobian

F = [1 0 1 0;
     0 1 0 1;
     0 0 1 0;
     0 0 0 1];


function H = h_function (x, u, n)
global B p1 p2 p3;
% measurement jacobian
px = x(1); py = x(2);

d1x = px - p1(1);
d1y = py - p1(2);
d1_x = d1x/sqrt( d1x^2 + d1y^2 );
d1_y = d1y/sqrt( d1x^2 + d1y^2 );

d2x = px - p2(1);
d2y = py - p2(2);
d2_x = d2x/sqrt( d2x^2 + d2y^2 );
d2_y = d2y/sqrt( d2x^2 + d2y^2 );

H = [d1_x d1_y 0 0;
     d2_x d2_y 0 0];


function y = state_propagation (x, u, n)
% (linear) state transition
global B p1 p2;

F = f_function(x, u, n);

y = F * x + B*u + n;

function z = state_measure (x, u, n)
global B p1 p2 p3;
% perform measurement
px = x(1); py = x(2);

d1x = px - p1(1);
d1y = py - p1(2);
d2x = px - p2(1);
d2y = py - p2(2);
dist1 = sqrt( d1x^2 + d1y^2 );
dist2 = sqrt( d2x^2 + d2y^2 );

z = [dist1; dist2] + n;


function pos = triangulate_measurement(p1, p2, x, z)
a = z(2);
b = z(1);
c = norm(p2 - p1);

phi = acos( (b^2 + c^2 - a^2)/(2*b*c) );

if (x(2) < 0)
    phi = -phi;
end

pos = [p1(1) + z(1)*cos(phi);
       p1(2) + z(1)*sin(phi)];

pos = real(pos);
