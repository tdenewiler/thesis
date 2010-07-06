% Clear everything and set up plot.
clc; clf; clear;

% Declare variables.
xstart = 20;
ystart = -10;
yawstart = 0;
xend = 0;
yend = 0;
yawend = 0;
step = 0.2;
time = 30;
maxu = 5;
maxw = 5;
borderpercent = 1.1;

% Parameters.
% Note that the gamma, h and k parameters are passed to f3 below as part
% of y only so that they don't have to be set again in the f3 function
% and can be modified here only.
gamma = 0.3;
h = 0.1;
k = 1;

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
xposmin = 0;
xposmax = 0;
yposmin = 0;
yposmax = 0;

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
    Vdot = -y(i,1) * u * cos(y(i,2)) + y(i,2) * ...
        (-w(i) + u(i) * sin(y(i,2)) / y(i,2) * ...
        (y(i,2) + h * theta) / y(i,1));

    % Set up the axes on the plot to show entire trajectory.
    if xpos(i) > xposmax
        xposmax = xpos(i);
    end
    if xpos(i) < xposmin
        xposmin = xpos(i);
    end
    if ypos(i) > yposmax
        yposmax = ypos(i);
    end
    if ypos(i) < yposmin
        yposmin = ypos(i);
    end
end

% Plot the trajectory to go from the start to the goal.
figure(1);
hold on;
axis([borderpercent * xposmin borderpercent * xposmax borderpercent * yposmin borderpercent * yposmax]);
axis equal
plot(xpos, ypos, 'b');
plot(xstart, ystart, 'go');
plot(xend, yend, 'ro');
title('Trajectory');
hold off;

% Plot x and y position versus time.
figure(2);
plot(t, xpos, 'r', t, ypos, 'b-.');
legend('\DeltaX', '\DeltaY', 'Location', 'Upper Right');
title('Position Errors');

% Plot the calculated input controls versus time.
figure(3);
plot(t, w, 'r', t, u, 'b-.');
legend('w', 'u', 'Location', 'Upper Right');
title('Velocities');

% Plot the derivative of the Lyapunov function versus time. This should be
% negative.
figure(4);
plot(t, Vdot, 'r');
legend('Vdot', 'Location', 'Upper Right');
title('Vdot');
