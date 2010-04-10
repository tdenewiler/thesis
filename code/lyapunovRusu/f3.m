function dx = f3(t, y)

% Initial conditions.
dx = [-(y(4) * cos(y(2))^2 * y(1));
    -y(6) * y(2) - y(4) * y(5) * cos(y(2)) * sin(y(2)) / y(2);
    y(4) * cos(y(2)) * sin(y(2));
    0;
    0;
    0];

