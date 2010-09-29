#! /usr/bin/env python

# Given inputs h and gamma this script will find an appropriate k to ensure that:
# (i) The linear system created as the angle errors (alpha, theta) approaches (0,0)
# is critically damped meaning that the two eigenvalues are the same.
# (ii) The negative of the eigenvalue is greater than gamma meaning that the angle
# errors will converge before the distance error.

from scipy import * # For eig()
from numpy import * # For array()
import sys # For argv

if len(sys.argv) != 3:
    print 'Not the correct input parameters. Usage is ./lyapunovFindEigs.py <h> <gamma>'
    sys.exit(1)

h = float(sys.argv[1])
gamma = float(sys.argv[2])

k = 2*gamma*sqrt(h)
print 'k =', k

A = array([[-1*k, -1*h*gamma],[gamma, 0]])
print 'A =', A
V,D = linalg.eig(A)
zeta = V[0]/V[1]
print 'V =', V
sigma = -V[0]

if zeta == 1:
    print 'System is critically damped since zeta =', zeta

if zeta.real > 1:
    print 'System is overdamped since zeta =', zeta

if zeta.real < 1:
    print 'System is underdamped since zeta =', zeta

if sigma < gamma:
    print 'Distance converges FASTER than angles since sigma =', sigma, '<', gamma

if sigma > gamma:
    print 'Distance converges SLOWER than angles since sigma =', sigma, '>', gamma
