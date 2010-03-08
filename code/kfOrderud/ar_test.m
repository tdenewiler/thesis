% Test of tracking model
clear all;
%close all;
%path('./ukf',path);

dt = 0.1;
x = [1 0.1 -1.1]';        % initial
P = [3 0 0;          % initial
     0 3 0;
     0 0 3];
z = 0;

x_ekf = x; x_ukf = x;
P_ekf = P; P_ukf = P;
 
u = 0; % no inputs

simulate_ssm('ar_model', x, z, dt);

for i=1:200
    % simulate process
    [x z] = simulate_ssm(u);
    xtrue(:,i) = x;
    zobs (:,i) = z;
    % estimate process
    [x_ekf P_ekf] = ekf(x_ekf, P_ekf, u, z, 'ar_model');
    [x_ukf P_ukf] = ukf(x_ukf, P_ukf, u, z, 'ar_model');
    xest_ekf(:,i) = x_ekf;
    xest_ukf(:,i) = x_ukf;
    diff_ekf(i) = (x - x_ekf)'*(x - x_ekf);
    diff_ukf(i) = (x - x_ukf)'*(x - x_ukf);
end

figure(1);
%subplot(1,2,1);
%plot(zobs);
%title 'Observed Position'

subplot(2,1,1); hold off
plot(zobs,'g' ); hold on
plot(xtrue(1,:), 'k' );5
plot(xest_ekf(1,:) );
plot(xest_ukf(1,:) ,'r');
title 'True and Estimated Position'

subplot(2,1,2); hold off;
plot(diff_ekf) ; hold on;
plot(diff_ukf, 'r');
title 'Error';

mean(diff_ekf)
mean(diff_ukf)
