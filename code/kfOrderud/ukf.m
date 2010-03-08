function [x_new P_new] = ukf (x, P, u, z, model_fun, as_idx, ao_idx)
% Unscented Kalman Filter code
% Copyright (c) Fredrik Orderud, 2005

% Permission to use and modify code in accordance to the BSD (Berkeley
% Software Distribution) licence is granted as long as the resulting code
% is non-viral (i.e. code MAY NOT BE GPL-ed).

if nargin < 6, as_idx = []; end
if nargin < 7, ao_idx = []; end


% Parameters
alpha = 1; %1e-3;
beta  = 2;
kappa = 0; %1e-3;

Q = feval(model_fun, 'Q', x, u, zeros(size(x)));
R = feval(model_fun, 'R', x, u, zeros(size(x)));

% Problem dimensions
states       = size(x,1);
observations = size(z,1);
wNoise       = size(Q,2);
vNoise       = size(R,2);
noises       = wNoise + vNoise;
L            = states + vNoise + wNoise;
nsp          = 2*L + 1;

s_idx = [];
for i=1:states
    if (sum(as_idx == i)) else
        s_idx(end+1) = i;
    end
end
o_idx = [];
for i=1:observations
    if (sum(ao_idx == i)) else
        o_idx(end+1) = i;
    end
end


% Step 1: Augment state & estimate covariance
tmp = [Q                    zeros(wNoise,vNoise); 
       zeros(vNoise,wNoise) R                   ]; % tmp
Pa  = [P                 zeros(states,noises); 
       zeros(noises,states) tmp                 ]; % augmented cov.

xa  = [x;
       zeros(noises,1)];                           % augmented state


% Step 2: Calculate Sigma points:
lambda = alpha^2 * (L + kappa) - L;
P_sq  = chol( (L + lambda) * Pa )';
P_sq2 = [zeros(L,1) P_sq -P_sq];

X_a(:,:)    = xa(:,ones(2*L+1,1)) + P_sq2(:,:);
X_a(as_idx,:) = amod(X_a(as_idx,:)); % angles moular 2pi


% Step 3: Calculate Weights
W_m(1) = lambda/(L + lambda);
W_c(1) = lambda/(L + lambda) + (1 - alpha^2 + beta);
for i=1:2*L
	W_m(i+1) = 1/(2*(L + lambda));
	W_c(i+1) = 1/(2*(L + lambda));
end


% Step 4: Predict
X_pre = zeros(states,2*L+1);
for i = 1:nsp
	X_pre(:,i) = feval(model_fun, 'S', X_a(1:states,i), u, X_a(states+1:states+wNoise,i));
end
x_pre1 = X_pre(as_idx,1);
X_pre(as_idx,1)     = 0;
X_pre(as_idx,2:end) = asub(X_pre(as_idx,2:end), x_pre1(:,ones(nsp-1,1)));

x_pre        = zeros(states, 1);
x_pre(s_idx) =       X_pre(s_idx,:) * W_m';
x_pre(as_idx)= amod( X_pre(as_idx,:)* W_m');

tmp1(s_idx,:)  =       X_pre(s_idx,:) - x_pre(s_idx, ones(nsp,1));
tmp1(as_idx,:) = asub( X_pre(as_idx,:), x_pre(as_idx, ones(nsp,1)) );

P_pre = W_c(1) * tmp1(:,1) * tmp1(:,1)' + W_c(2) * tmp1(:,2:nsp) * tmp1(:,2:nsp)';

x_pre(as_idx) = aadd( x_pre(as_idx,:), x_pre1); % add back


Z_pre = zeros(observations,nsp);
for i=1:nsp
	Z_pre(:,i) = feval(model_fun, 'M', X_pre(:,i), u, X_a(states+wNoise+1:end,i));
end

%%%%%%%%%%%%%%%%%%%%

z_pre1 = Z_pre(ao_idx,1);
Z_pre(ao_idx,1) = 0;
Z_pre(ao_idx,2:end) = asub( Z_pre(ao_idx,2:end), z_pre1(:,ones(nsp-1,1)) );

z_pre         = zeros(observations,1);
z_pre(o_idx)  = Z_pre(o_idx,:) * W_m';
z_pre(ao_idx) = amod( Z_pre(ao_idx,:) * W_m' );

tmp2 = Z_pre - z_pre(:,ones(nsp,1));
tmp2(ao_idx,:) = asub( Z_pre(ao_idx,:), z_pre(ao_idx,ones(nsp,1)) );

z_pre(ao_idx) = aadd( z_pre(ao_idx,:), z_pre1); % add back

% Step 5: Measurement Update
P_zz = W_c(1) * tmp2(:,1) * tmp2(:,1)' + W_c(2) * tmp2(:,2:nsp) * tmp2(:,2:nsp)';
P_xz = W_c(1) * tmp1(:,1) * tmp2(:,1)' + W_c(2) * tmp1(:,2:nsp) * tmp2(:,2:nsp)';

K = P_xz / P_zz;

inov         = z - z_pre;
inov(ao_idx) = amod( inov(ao_idx) );

x_new         = x_pre + K * inov;
x_new(as_idx) = amod( x_new(as_idx) );

P_new = P_pre - K * P_zz * K';

% Finished :)




function C = aadd (A, B)
C = A + B;
idx1 = C > pi;
idx2 = C < -pi;
C(idx1) = C(idx1) - 2*pi;
C(idx2) = C(idx2) + 2*pi;


function C = asub (A, B)
C = A - B;
idx1 = C > pi;
idx2 = C < -pi;
C(idx1) = C(idx1) - 2*pi;
C(idx2) = C(idx2) + 2*pi;

function C = amod (A)
% calculates C = A % 2pi; in range(p, -pi)
C = A/pi;
idx1 = C >  1;
idx2 = C < -1;
C(idx1) = C(idx1) - 2 * round( C(idx1)/2);
C(idx2) = C(idx2) + 2 * round(-C(idx2)/2);
C = C*pi;
