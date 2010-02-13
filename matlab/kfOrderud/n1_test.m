% Test of tracking model
clear all;
%close all;
%path('./ukf',path);

dt = 0.1;
x = 1;        % initial
P = 1;
z = 0;

x_ekf = x; x_ukf = x;
P_ekf = P; P_ukf = P;
 
u = 0; % no inputs

simulate_ssm('n1_model', x, z, dt);

for i=1:60
    % simulate process
    [x z] = simulate_ssm(u);
    xtrue(:,i) = x;
    zobs (:,i) = z;
    % estimate process
    [x_ekf P_ekf] = ekf(x_ekf, P_ekf, u, z, 'n1_model');
    [x_ukf P_ukf] = ukf(x_ukf, P_ukf, u, z, 'n1_model');
    xest_ekf(:,i) = x_ekf;
    xest_ukf(:,i) = x_ukf;
    diff_ekf(i) = (x - x_ekf)'*(x - x_ekf);
    diff_ukf(i) = (x - x_ukf)'*(x - x_ukf);
end

plot_results(xtrue, zobs, xest_ekf, xest_ukf);

