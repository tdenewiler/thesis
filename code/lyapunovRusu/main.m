% Clear everything and set up plot.
clc; clf; clear;
figure(1);
hold off;
plot(0, 0);
hold on;

% Declare variables.
xstart = -1;
ystart = 1;
yawstart = 0;
xend = 0;
yend = 0;
yawend = 0;
step = 0.2;
time = 10;
maxu = 5;
maxw = 5;
circlesize = 0.02;
if xstart > xend
    if ystart > yend
        axis([xend-step xstart+step yend-step ystart+step]);
    else
        axis([xend-step xstart+step ystart-step yend+step]);
    end
else
    if ystart > yend
        axis([xstart-step xend+step yend-step ystart+step]);
    else
        axis([xstart-step xend+step ystart-step yend+step]);
    end
end
axis equal;

% Parameters.
% Note that the gamma, h and k parameters are passed to f3 below as part
% of y only so that they don't have to be set again in the f3 function
% and can be modified here only.
gamma = 1;
h = 1;
k = 6;

% Calculate the initial errors for position and heading.
[e, theta, a] = CalcETA(xstart-xend, ystart-yend, yawstart-yawend);

% Simulate the kinematics for the trajectory to get the state variables
% through time. The state variables are:
% y(1) = e, y(2) = alpha and y(3) = theta.
[t, y] = ode45(@f3, [0 time], [e a theta gamma h k]);

% Set up vectors to hold x and y positions, yaw speed and velocity of robot.
xpos = zeros(1,length(t));
ypos = zeros(1,length(t));
w = zeros(1,length(t));
u = zeros(1,length(t));
Vdot = zeros(1,length(t));

% At each time step calculate the forward velocity and angular rate commands
% to issue to the robot. Note that Vdot should be <= 0 is a sanity check.
for i = 1:length(t)
    xpos(i) = sqrt(y(i,1)^2 * (cos(y(i,3))^2));
    ypos(i) = xpos(i) * tan(y(i,3));
    u(i) = gamma * cos(y(i,2)) * y(i,1);
    if abs(u(i)) > maxu
        u(i) = sign(u(i)) * maxu;
    end
    w(i) = k * y(i,2) + gamma * (cos(y(i,2)) * ...
        sin(y(i,2)) / y(i,2)) * (y(i,2) + h * y(i,3));
    if abs(w(i)) > maxw
        w(i) = sign(w(i)) * maxw;
    end
    Vdot = -e * u * cos(y(i,2)) + y(i,2) * ...
        (-w(i) + u(i) * sin(y(i,2)) / y(i,2) * ...
        (y(i,2) + h * theta) / y(i,1));
end

% Plot the trajectory to go from the start to the goal.
plot(ypos, xpos, 'r');
[latc, lonc] = scircle1(xstart, ystart, circlesize);
plot(latc, lonc, 'b');
[latc, lonc] = scircle1(xend, yend, circlesize);
plot(latc, lonc, 'b');
hold off;

% Plot x and y position versus time.
figure(2);
plot(t, xpos, 'r', t, ypos, 'b-.');
legend('X', 'Y', 'Location', 'Best');

% Plot the calculated input controls versus time.
figure(3);
plot(t, w, 'r', t, u, 'b-.');
legend('w', 'u', 'Location', 'Best');

% Plot the derivative of the Lyapunov function versus time. This should be
% negative.
figure(4);
plot(t, Vdot, 'r');
legend('Vdot', 'Location', 'Best');