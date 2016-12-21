function dx = f3(t, y)

% ODEs based on robot kinematics.
% \dot{e} = -\gamma e \cos^2\alpha
% \dot{\alpha} = -k\alpha - \gamma h \frac{\cos\alpha\sin\alpha}{\alpha}
% \dot{\theta} = \gamma\cos\alpha\sin\alpha

% Variables:
% y(1) = e
% y(2) = alpha
% y(3) = theta
% y(4) = gamma
% y(5) = h
% y(6) = k

dx = [-(y(4) * cos(y(2))^2 * y(1));
      -y(6) * y(2) - y(4) * y(5) * cos(y(2)) * sin(y(2)) / y(2);
      y(4) * cos(y(2)) * sin(y(2));
      0;
      0;
      0];
