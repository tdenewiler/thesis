function [mse_radar_ekf mse_radar_ukf mse_tria_ekf mse_tria_ukf] = comparison_test ()
% Test of tracking model
clear all;
%close all;
%path('./ukf',path);

N = 80;
dt = 0.1;
x = [-200 200 4 0]';        % initial
x_radar_ekf = x; x_radar_ukf = x;
x_tria_ekf  = x; x_tria_ukf  = x;
z = [0; 0];

P = [1 0 0 0;          % initial
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
P_radar_ekf = P; P_radar_ukf = P;
P_tria_ekf  = P; P_tria_ukf  = P;

u = 0; % no inputs

simulate_ssm('radar_model', x, z, dt);

diff_radar_ekf = zeros(N,1);
diff_radar_ukf = zeros(N,1);
diff_tria_ekf  = zeros(N,1);
diff_tria_ukf  = zeros(N,1);

for i=1:N
    % simulate process
    [x z_radar] = simulate_ssm(u);
    z_tria = simulate_ssm('triangulation_model', x);
    %xtrue(:,i) = x;
    %zobs (:,i) = z;  
    %ztria(:,i) = triangulation_model('tria', x, z);
    
    % estimate process
    [x_radar_ekf P_radar_ekf] = ekf(x_radar_ekf, P_radar_ekf, u, z_radar, 'radar_model', [], []);
    [x_radar_ukf P_radar_ukf] = ukf(x_radar_ukf, P_radar_ukf, u, z_radar, 'radar_model', [], []);
    diff_radar_ekf(i) = (x - x_radar_ekf)'*(x - x_radar_ekf);
    diff_radar_ukf(i) = (x - x_radar_ukf)'*(x - x_radar_ukf);
    
    [x_tria_ekf P_tria_ekf] = ekf(x_tria_ekf, P_tria_ekf, u, z_tria, 'triangulation_model', [], []);
    [x_tria_ukf P_tria_ukf] = ukf(x_tria_ukf, P_tria_ukf, u, z_tria, 'triangulation_model', [], []);
    diff_tria_ekf(i) = (x - x_tria_ekf)'*(x - x_tria_ekf);
    diff_tria_ukf(i) = (x - x_tria_ukf)'*(x - x_tria_ukf);
end

mse_radar_ekf = mean(diff_radar_ekf);
mse_radar_ukf = mean(diff_radar_ukf);
mse_tria_ekf  = mean(diff_tria_ekf);
mse_tria_ukf  = mean(diff_tria_ukf);

%plot_results(xtrue, ztria, est_ekf, est_ukf);

