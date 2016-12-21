function [e, theta, alpha] = CalcETA(x, y, phi)
% Calculate the initial trajectory for the robot.
e = sqrt(x^2 + y^2);
theta = atan2(-x, -y);
alpha = theta - phi;