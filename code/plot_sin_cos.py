#! /usr/bin/env python
"""Plot custom function."""

from math import cos, pi, sin
import numpy as np
import matplotlib.pyplot as plt # pylint: disable=import-error

# pylint: disable=invalid-name

saveplots = 1
xres = 0.001
x = np.arange(-pi, pi+xres, xres)
ysin = sin(x)
ycos = cos(x)
ysincos = sin(x)*cos(x)
yx0 = x * 0 # Horizontal line for x-axis
yx = x * 1 # To compare angle to other values
plt.plot(x, ysin)
plt.hold(True)
plt.plot(x, ycos)
plt.plot(x, ysincos)
plt.plot(x, yx)
plt.plot(x, yx0, 'k')
plt.axis('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.title(r'Useful Trigonometric Functions of $\alpha$')
plt.legend((r'$\sin(\alpha)$', r'$\cos(\alpha)$', r'$\cos(\alpha)*\sin(\alpha)$',
            r'$\alpha$'), 'upper left')

if saveplots:
    plt.savefig("../images/plotSinCos.svg")
    plt.savefig("../images/plotSinCos.png", dpi=600)

plt.show()
plt.close()
