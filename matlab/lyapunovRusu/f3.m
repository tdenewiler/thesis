function dx = f3(t, y)
% Parameters.
gamma = 1;
h = 1;
k = 6;

% Dynamics of the state variables.
dx = [-(gamma * cos(y(2))^2 * y(1));
    -k * y(2) - gamma * h * (cos(y(2)) *
		sin(y(2)) / y(2));
    gamma * cos(y(2)) * sin(y(2))];
