#! /usr/bin/env python3
"""Plot custom function."""

from math import pi
import numpy as np
import matplotlib.pyplot as plt

# pylint: disable=invalid-name

saveplots = 1
resolution = 0.001
x = np.arange(-pi, pi+resolution, resolution)
ysin = np.sin(x)
ycos = np.cos(x)
ysincos = np.sin(x) * np.cos(x)
yx0 = x * 0  # Horizontal line for x-axis
yx = x * 1  # To compare angle to other values

plt.plot(x, ysin, label=r'$\sin(\alpha)$')
plt.plot(x, ycos, label=r'$\cos(\alpha)$')
plt.plot(x, ysincos, label=r'$\cos(\alpha)*\sin(\alpha)$')
plt.plot(x, yx, label=r'$x$')
plt.plot(x, yx0, 'k', label=r'$\alpha$')
plt.axis('equal')
plt.xlabel('x')
plt.ylabel('y')
plt.title(r'Useful Trigonometric Functions of $\alpha$')
plt.legend()

if saveplots:
    plt.savefig("../images/plotSinCos.svg")
    plt.savefig("../images/plotSinCos.png", dpi=600)

plt.show()
plt.close()
