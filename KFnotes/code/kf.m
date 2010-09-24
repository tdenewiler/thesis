% Kalman filter example program for differential drive robots.
% Perfect measurements are available for all the states except for yaw.
% Yaw has three sensors that have adjustable amounts of noise and drift.
% Output is RMS errors for position and heading plus plots showing some of the state variables.
clear all;
pngres = '-r600';
saveimages = 0;

% Number of time steps.
N = 100;

% Sensor behavior variables. At least one Noise value must be > 0 else the inverse in the gain
% calculation is ill-conditioned.
Yaw1Noise = 1 * pi / 180;
Yaw2Noise = 5 * pi / 180;
Yaw1Drift = 0;
Yaw2Drift = 0;

% BEGIN {Data Generation}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This part is secret business and involves the generation of the data.
% The data arise in the real world and their generation is invisible to us.

% Set up ground truth for state variables and sensor measurements.
% The state vector is x = [X Y Z V theta(pitch) phi(roll) psi(yaw) w(ang rate)]
x = [];
y = [];
R = 100; % Path is a circle, radius R.
angle = 0;
for i = 1:N+1
    xi(1) = R*cos(angle);
    xi(2) = R*sin(angle);
    xi(3) = 0;
    xi(4) = 1;
    xi(5) = 0;
    xi(6) = 0;
    xi(7) = -1 * atan2(xi(2),xi(1));
    xi(8) = 4*xi(4)/R;
    x = [x; xi];
    angle = angle + 2*pi/N;
    
    % Add noise and drift to IMU yaw measurements.
    psi1(i) = xi(7) + Yaw1Noise*randn + i*Yaw1Drift;
    psi2(i) = xi(7) + Yaw2Noise*randn + i*Yaw2Drift;

    % Set up measurement vector with ground truth except for multiple heading measurements.
    yi = [xi(1) xi(2) xi(3) xi(4) xi(5) xi(6) psi1(i) psi2(i) xi(8)];
    y = [y; yi];
end

% Now we have the data vector y, which is all that will be passed into the real world.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% END {Data Generation}

% Measurement transition matrix. #cols = #states, #rows = #measurements.
H = [eye(8); zeros(1,8)];
% Set up H to reflect that yaw (column 7) is measured three times, all others measured once.
H(7,7) = 1;
H(8,7) = 1;
H(9,8) = 1;

% State covariance matrix.
Q = eye(8);

% Measurement covariance matrix.
R = zeros(9);
R(7,7) = Yaw1Noise;
R(8,8) = Yaw2Noise;

% Initialization of state estimate and filter covariance.
xh = x(1,:)';
P = Q;
P(7,7) = max(Yaw1Noise,Yaw2Noise);
xhat = xh;
Pp = diag(P);
Kk = [];

% Run the KF equations.
for i = 1:N
    % State transition matrix based on dynamics.
    F = [1 0 0 cos(xh(7))*cos(xh(5)) 0 0 0 0;
         0 1 0 sin(xh(7))*cos(xh(5)) 0 0 0 0;
         0 0 1 -sin(xh(5)) 0 0 0 0;
         0 0 0 1 0 0 0 0;
         0 0 0 0 1 0 0 -sin(xh(6));
         0 0 0 0 0 1 0 tan(xh(5))*cos(xh(6));
         0 0 0 0 0 0 1 cos(xh(6))/cos(xh(5));
         0 0 0 0 0 0 0 1];

    % Prediction update step.
    xh = F*xh;
    Pm = F*P*F'+Q;

    % Measurement update step.
    K = Pm*H'/(H*Pm*H'+R);
    P = (eye(8)-K*H)*Pm;
    xh = (eye(8)-K*H)*xh + K*y(i+1,:)';

    % Save state estimate, state covariance and gains.
    xhat = [xhat xh];
    Pp = [Pp diag(P)];
    Kk = [Kk K];
end

% Plot the yaw estimate.
figure(1)
hold on;
plot(x(:,7), 'g');
plot(psi1, 'b');
plot(psi2, 'r');
plot(xhat(7,:), 'kx');
%errorbar(xhat(7,:), Pp(7,:), 'kx');
title('Yaw');
xlabel('Time (s)');
ylabel('Yaw (radians)');
legend('Actual','Sensor 1','Sensor 2','KF');
axis equal;
hold off;
if saveimages
    print('-f1', pngres, '-dpng', '../images/kfSimYaw.png');
end

% Plot the yaw estimate zoomed in.
figure(2)
hold on;
plot(x(:,7), 'g');
plot(psi1, 'b');
plot(psi2, 'r');
plot(xhat(7,:), 'kx');
%errorbar(xhat(7,:), Pp(7,:), 'kx');
title('Yaw');
xlabel('Time (s)');
ylabel('Yaw (radians)');
legend('Actual','Sensor 1','Sensor 2','KF');
axis([55.5 61 0.5 4]);
axis equal;
hold off;
if saveimages
    print('-f2', pngres, '-dpng', '../images/kfSimYawZoom.png');
end

% Plot the actual and estimated position.
figure(3)
hold on;
plot(x(:,1), x(:,2), 'g');
plot(xhat(1,:), xhat(2,:), 'kx');
title('Position');
xlabel('X (m)');
ylabel('Y (m)');
legend('Actual','KF');
axis([-105 105 -105 105]);
axis equal;
hold off;
if saveimages
    print('-f3', pngres, '-dpng', '../images/kfSimPosition.png');
end

% Calculate the RMS errors.
epos = 0;
eyaw = 0;
for i=1:N
    epos = epos + (x(i,1)-xhat(1,i))^2 + (x(i,2)-xhat(2,i))^2;
    eyaw = eyaw + (x(i,7)-xhat(7,i))^2;
end
epos = sqrt(epos/N);
eyaw = sqrt(eyaw/N);
stringpos = sprintf('RMS position error = %f meters\n', epos);
stringyaw = sprintf('RMS heading error = %f degrees\n', eyaw*180/pi);
stringcov = sprintf('Yaw state estimate covariance = %f degrees\n', P(7,7)*180/pi);
disp(stringpos);
disp(stringyaw);
disp(stringcov);
