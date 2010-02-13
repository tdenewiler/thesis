%function [mse_ekf mse_ukf] = radar_test ()
% Test of tracking model
clear all;
%close all;
%path('./ukf',path);

N = 80;
dt = 0.1;
x = [-200 200 4 0]';        % initial
x_ekf = x;
x_ukf = x;
z = [0; 0];

P = [1 0 0 0;          % initial
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
P_ekf = P;
P_ukf = P;
 
u = 0; % no inputs

simulate_ssm('tracking_model', x, z, dt);

diff_ekf = zeros(N,1);
diff_ukf = zeros(N,1);
for i=1:N
    % simulate process
    [x z] = simulate_ssm(u);
    xtrue(:,i) = x;
    zobs (:,i) = z;
    % estimate process
    [x_ekf P_ekf] = ekf(x_ekf, P_ekf, u, z, 'tracking_model', [], 2);
    [x_ukf P_ukf] = ukf(x_ukf, P_ukf, u, z, 'tracking_model', [], 2);
    xest_ekf(:,i) = x_ekf;
    xest_ukf(:,i) = x_ukf;
    diff_ekf(i) = (x - x_ekf)'*(x - x_ekf);
    diff_ukf(i) = (x - x_ukf)'*(x - x_ukf);
end

for i=1:N
    obs(1,i) = cos(zobs(2,i))*zobs(1,i);
    obs(2,i) = sin(zobs(2,i))*zobs(1,i);
end
plot_results(xtrue, obs, xest_ekf, xest_ukf);


