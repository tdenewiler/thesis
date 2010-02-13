function plot_results (xtrue, ztria, est_ekf, est_ukf)

if (size(xtrue,1) > 1)
    diff_ekf = sum((xtrue - est_ekf).^2);
else
    diff_ekf = (xtrue - est_ekf).^2;
end

if (size(xtrue,1) > 1)
    diff_ukf = sum((xtrue - est_ukf).^2);
else
    diff_ukf = (xtrue - est_ukf).^2;
end


figure(1);

subplot(1,1,1); hold off 
plot(ztria(1,:), ztria(2,:), 'kx' ); hold on
plot(xtrue(1,:), xtrue(2,:), 'k:' ); 
%plot(est_ekf(1,:), est_ekf(2,:), 'k' );
plot(est_ukf(1,:), est_ukf(2,:) ,'k');
title 'Observed, true and estimated trajectory'
axis equal;
legend('observations','true trajectory','estimated trajectory');

%subplot(1,2,2); hold off;
%plot(diff_ekf) ; hold on;
%plot(diff_ukf, 'r');
%title 'Error';

mean(diff_ekf)
mean(diff_ukf)
