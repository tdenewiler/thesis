clc; clf; clear;
hold off;
plot(0, 0);
hold on;
axis([-1.2 0.2 -0.2 1.2]);

% Parameters.
gamma = 1;
h = 1;
k = 6;

% Generate an initial trajectory to follow.
[e, th, a] = CalcETA(1, -1, 0);

% Simulate the physics for the trajectory to get the state variables
% through time. The state variables are y(1) = e, y(2) = alpha and
% y(3) = theta. This is basically planner output.
[t, y] = ode45(@f3, [0 10], [e a th]);

% Set up vectors to hold x and y position, heading and velocity of
% robot.
xpos = zeros(length(t));
ypos = zeros(length(t));
w = zeros(length(t));
u = zeros(length(t));

% At each time step calculate the forward velocity and angular rate
% commands to issue to the robot.
for i = 1:length(t)
    xpos(i) = sqrt(y(i,1)^2 * (cos(y(i,3))^2));
%     xpos(i) = sqrt(y(i,1)^2 / (1 + tan(y(i,3))^2));
    ypos(i) = xpos(i) * tan(y(i,3));
    w(i) = k * y(i,2) + gamma * (cos(y(i,2)) * ...
        sin(y(i,2)) / y(i,2)) * (y(i,2) + h * y(i,3));
    u(i) = gamma * cos(y(i,2)) * y(i,1);
end

% Plot the trajectory to go from the start to the goal.
plot(ypos, xpos, 'r');
[latc, longc] = scircle1(-1, 1, 0.02);
plot(latc, longc, 'b');
[latc, longc] = scircle1(0, 0, 0.02);
plot(latc, longc, 'b');

% Plot the calculated input controls versus time.
pause;
hold off;
plot(t, w, 'r', t, u, 'b');