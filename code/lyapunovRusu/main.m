% Clear everything and set up plot.
clc; clf; clear;
hold off;
plot(0, 0);
hold on;

% Declare variables.
xstart = -2;
ystart = 1;
yawstart = 0;
xend = 0;
yend = 0;
yawend = 0;
step = 0.2;
time = 20;
circlesize = 0.02;
axis([xstart-step step -step ystart+step]);

% Parameters.
gamma = 1;
h = 1.1;
k = 5;

% Calculate the initial errors for position and heading.
[e, th, a] = CalcETA(xstart, ystart, yawstart);

% Simulate the physics for the trajectory to get the state variables through
% time. The state variables are y(1) = e, y(2) = alpha and y(3) = theta. This
% is local planner output between via points based on the Lyapunov function.
[t, y] = ode45(@f3, [0 time], [e a th]);

% Set up vectors to hold x and y positions, yaw speed and velocity of robot.
xpos = zeros(length(t));
ypos = zeros(length(t));
w = zeros(length(t));
u = zeros(length(t));

% At each time step calculate the forward velocity and angular rate commands
% to issue to the robot.
for i = 1:length(t)
    dou = 1;
    dow = 1;
    xpos(i) = sqrt(y(i,1)^2 * (cos(y(i,3))^2));
    ypos(i) = xpos(i) * tan(y(i,3));
    u(i) = gamma * cos(y(i,2)) * y(i,1);
    w(i) = k * y(i,2) + gamma * (cos(y(i,2)) * ...
        sin(y(i,2)) / y(i,2)) * (y(i,2) + h * y(i,3));
end

% Plot the trajectory to go from the start to the goal.
plot(ypos, xpos, 'r');
[latc, lonc] = scircle1(xstart, ystart, circlesize);
plot(latc, lonc, 'b');
[latc, lonc] = scircle1(xend, yend, circlesize);
plot(latc, lonc, 'b');

% Plot the calculated input controls versus time.
pause;
hold off;
plot(t, w, 'r', t, u, 'b');
legend('w Red','u Blue','Location','Best');
