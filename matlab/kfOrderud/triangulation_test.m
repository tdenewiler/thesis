%function [mse_ekf mse_ukf] = triangulation_test ()
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

simulate_ssm('triangulation_model', x, z, dt);

diff_ekf = zeros(N,1);
diff_ukf = zeros(N,1);
for i=1:N
    % simulate process
    [x z] = simulate_ssm(u);
    xtrue(:,i) = x;
    %zobs (:,i) = z;  
    ztria(:,i) = triangulation_model('tria', x, z);
    
    % estimate process
    [x_ekf P_ekf] = ekf(x_ekf, P_ekf, u, z, 'triangulation_model', [], []);
    [x_ukf P_ukf] = ukf(x_ukf, P_ukf, u, z, 'triangulation_model', [], []);
    est_ekf(:,i) = x_ekf;
    est_ukf(:,i) = x_ukf;
    diff_ekf(i) = (x - x_ekf)'*(x - x_ekf);
    diff_ukf(i) = (x - x_ukf)'*(x - x_ukf);
end

mse_ekf  = mean(diff_ekf);
mse_ukf = mean(diff_ukf);

plot_results(xtrue, ztria, est_ekf, est_ukf);

