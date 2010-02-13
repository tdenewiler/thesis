function [x P] = ekf (x, P, u, z, model_fun, as_idx, ao_idx)

if nargin < 6, as_idx = []; end
if nargin < 7, ao_idx = []; end


% initialize Kalman filter
% assume linearized x_k+1 = F * x_k + B * u + noise(Q)
%                   z_k   = H * x_k +         noise(R)
s_noise = zeros(size(x));
m_noise = zeros(size(z));
F = feval(model_fun, 'F', x, u, s_noise);
Q = feval(model_fun, 'Q', x, u, s_noise);
R = feval(model_fun, 'R', x, u, m_noise);
H = feval(model_fun, 'H', x, u, m_noise);

% Predict next time-step
% x_p = F * x +  B * u;
x_p = feval(model_fun, 'S', x, u, s_noise);
%x_p(as_dx) = amod( x_p(as_dx) );
P_p = F * P * F' + Q;

H = feval(model_fun, 'H', x_p, u, m_noise);
% Update estimate with measurements
S = H * P_p * H' + R;
K = P_p * H' * inv(S);

y_p = feval(model_fun, 'M', x_p, u, m_noise);
%y_p(ao_dx) = amod( y_p(ao_dx) );

inov         = z - y_p;
inov(ao_idx) = amod( inov(ao_idx) );
x            = x_p + K * inov;
x(as_idx)    = amod( x(as_idx) );

P = P_p - K * H * P_p;




function C = amod (A)
% calculates C = A % 2pi; in range(p, -pi)
C = A/pi;

idx1 = C >  1;
idx2 = C < -1;

C(idx1) = C(idx1) - 2 * round( C(idx1)/2);
C(idx2) = C(idx2) + 2 * round(-C(idx2)/2);

C = C*pi;
