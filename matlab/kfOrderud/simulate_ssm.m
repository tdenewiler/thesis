function [x z] = simulate_ssm (a, b, c, d)
% Simulation of State Space Models with gaussian state- and measurement noise

if     nargin == 4, [x z] = linsim_init(a, b, c, d);
elseif nargin == 2,      x = linsim_measure(a, b);
elseif nargin == 1, [x z] = linsim_update(a);
else                disp 'ERROR in simulate_ssm';
end

function [x z] = linsim_init (model_fun, xinit, zinit, d_t)
% initialization
global MOD_FUN x z Q_f R_f dt;

MOD_FUN = model_fun;
x = xinit;
z = zinit;
dt = d_t;
Q_f = chol( feval(MOD_FUN, 'Q') ); % state prop. noise
R_f = chol( feval(MOD_FUN, 'R') ); % measurement noise


function [x z] = linsim_update (u)
% update & measure
global MOD_FUN x z Q_f R_f dt;

% propagate state
s_n = Q_f * randn(length(x),1);
x = feval(MOD_FUN, 'S', x, u, s_n);

% measure
m_n = R_f * randn(length(z),1);
z = feval(MOD_FUN, 'M', x, u, m_n);


function [z] = linsim_measure (m_fun, x)
% only perfom a measurement
global u;

% measure
R = chol( feval(m_fun, 'R') );
m_n = R * randn(size(R,1),1);
z = feval(m_fun, 'M', x, u, m_n);
