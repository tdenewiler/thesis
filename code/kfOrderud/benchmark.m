% EKF / UKF benchmark test
clear all;

N = 10000;
%disp 'Benchmark begins...';
%
%for i=1:N
%    [radar_ekf(i) radar_ukf(i) tria_ekf(i) tria_ukf(i)] = comparison_test ();
%end
%disp 'Finished';
%save 'benchmark_10k' radar_ekf radar_ukf tria_ekf tria_ukf;


load 'benchmark_10k' ;
mean(radar_ekf)
mean(radar_ukf)
mean(tria_ekf)
mean(tria_ukf)

errmax_radar = 1e3;
radar_ekf(radar_ekf > errmax_radar) = errmax_radar;
radar_ukf(radar_ukf > errmax_radar) = errmax_radar;
errmax_tria = 1e3; %4e3;
tria_ekf(tria_ekf > errmax_tria) = errmax_tria;
tria_ukf(tria_ukf > errmax_tria) = errmax_tria;

%m_tria_ekf = mean(tria_ekf)
%m_tria_ukf = mean(tria_ukf)
%v_tria_ekf = var(tria_ekf)/N;
%v_tria_ukf = var(tria_ukf)/N;
%m_tria_ekf - m_tria_ukf
%v_tria_ekf + v_tria_ukf

%m_radar_ekf = mean(radar_ekf)
%m_radar_ukf = mean(radar_ukf)
%v_radar_ekf = var(radar_ekf)/N;
%v_radar_ukf = var(radar_ukf)/N;
%m_radar_ekf - m_radar_ukf
%v_radar_ekf + v_radar_ukf

figure(1);
[x_rad_ekf y_rad_ekf] = logx_hist(radar_ekf,100, errmax_radar);
[x_rad_ukf y_rad_ukf] = logx_hist(radar_ukf,100, errmax_radar);
[x_tri_ekf y_tri_ekf] = logx_hist(tria_ekf,100, errmax_tria);
[x_tri_ukf y_tri_ukf] = logx_hist(tria_ukf,100, errmax_tria);

subplot(2,1,1); hold off;
semilogx(x_rad_ekf, y_rad_ekf,'k:'); hold on;
semilogx(x_rad_ukf, y_rad_ukf,'k--');
axis([10 1000 0 1000]);
title 'Error distribution for radar tracking';
legend('ekf','ukf');
subplot(2,1,2); hold off;
semilogx(x_tri_ekf, y_tri_ekf,'k:'); hold on;
semilogx(x_tri_ukf, y_tri_ukf,'k--');
axis([10 1000 0 1000]);
title 'Error distribution for triangulation tracking';
legend('ekf','ukf')
