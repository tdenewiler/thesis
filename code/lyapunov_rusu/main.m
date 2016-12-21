% Clear everything and set up plot.
clc; clf; clear;

% Declare variables.
xstart = 1;
ystart = -1;
yawstart = 0;
xend = 0;
yend = 0;
yawend = 0;
time = 10;
plotpad = 1.1;

% Gains.
% Note that the gamma, h and k parameters are passed to f3 below as part
% of y only so that they don't have to be set again in the f3 function
% and can be modified here only.
gamma = 0.24;
h = 0.1;
k = 0.25;

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

% At each time step calculate the linear and angular velocity commands.
% Note that Vdot should be <= 0 is a sanity check.
for i = 1:length(t)
    xpos(i) = sqrt(y(i,1)^2 * (cos(y(i,3))^2));
    ypos(i) = xpos(i) * tan(y(i,3));
    u(i) = gamma * cos(y(i,2)) * y(i,1);
    w(i) = k * y(i,2) + gamma * (cos(y(i,2)) * ...
        sin(y(i,2)) / y(i,2)) * (y(i,2) + h * y(i,3));
    Vdot = -y(i,1) * u * cos(y(i,2)) + y(i,2) * ...
        (-w(i) + u(i) * sin(y(i,2)) / y(i,2) * ...
        (y(i,2) + h * theta) / y(i,1));
end

% Find the position limits to use when plotting.
xposmin = min(xpos);
xposmax = max(xpos);
yposmin = min(ypos);
yposmax = max(ypos);

% Plot the trajectory to go from the start to the goal.
figure(1);
hold on;
axis([plotpad*xposmin plotpad*xposmax plotpad*yposmin plotpad*yposmax]);
axis equal
plot(xpos, ypos, 'b');
plot(xstart, ystart, 'go');
plot(xend, yend, 'ro');
title('Trajectory');
hold off;

% Plot x and y position versus time.
figure(2);
plot(t, xpos, 'r', t, ypos, 'b-.');
legend('\DeltaX', '\DeltaY');
title('Position Errors');

% Plot the calculated input controls versus time.
figure(3);
plot(t, w, 'r', t, u, 'b-.');
legend('w', 'u');
title('Velocities');

% Plot the errors returned by the ODE solver.
figure(4);
plot(t, y(:,1), t, y(:,2), t, y(:,3));
legend('e', '\alpha', '\theta');
title('Errors');

% Plot the derivative of the Lyapunov function versus time. This should be
% negative.
figure(5);
plot(t, Vdot, 'r');
legend('Vdot');
title('Vdot');
