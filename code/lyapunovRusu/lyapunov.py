#! /usr/bin/env python

# Simulation of differential drive robot using model-based controller
# based on Lyapunov stability theory.

from scipy import *
from scipy.integrate import odeint
from matplotlib import *
from pylab import *
from math import *
import time

pngres = 100
saveimages = 0

#####################################################################
# kinematicsODE()
# Solves the differential equations from robot kinematics equations
# to generate a trajectory that the robot will drive.
#
# ODEs
# \dot{e} = -\gamma e \cos^2\alpha
# \dot{\alpha} = -k\alpha - \gamma h \frac{\cos\alpha\sin\alpha}{\alpha}
# \dot{\theta} = \gamma\cos\alpha\sin\alpha
#
# Variables:
# y(0) = e
# y(1) = alpha
# y(2) = theta
def kinematicsODE(y, t, gamma, h, k):
    edot = -gamma * y[0] * cos(y[1])**2
    alphadot = -k * y[1] - gamma * h * y[2] * cos(y[1]) * sin(y[1]) / y[1]
    thetadot = gamma * cos(y[1]) * sin(y[1])
    
    return [edot, alphadot, thetadot]
# end kinematicsODE()


#####################################################################
# Main.

# Start pose.
xstart = 1
ystart = -1
yawstart = 0
xend = -25
yend = 3
yawend = 0

# Time to run simulation.
tend = 60
tinc = 0.1
t = arange(0, tend, tinc)

# Gains.
gamma = 0.25
h = 0.33
k = 0.30

# Calculate the initial errors for position and heading.
dxe = xend - xstart
dye = yend - ystart
thetastar = yawend
psi = yawstart
e = sqrt((dxe)**2 + (dye)**2)
thetae = atan2(dye, dxe)
phi = thetastar - psi
theta = thetae - thetastar
alpha = theta - phi
y0 = [e, theta, alpha]
print 'Initial errors are e =', e,'m, alpha =', alpha,'rad, theta =', theta,'rad, thetae =', thetae,'rad'

# Simulate the kinematics for the trajectory to get the state variables
# through time. The state variables are:
# y[1] = e, y[2] = alpha and y[3] = theta.
start = time.time()
y = odeint(kinematicsODE, y0, t, (gamma, h, k))
elapsed = time.time() - start
print 'ODE solver took %0.5f seconds' % elapsed

# At each time step calculate the linear and angular velocity commands.
# Note that Vdot should be <= 0 is a sanity check.
xpos = zeros((tend/tinc,1))
ypos = zeros((tend/tinc,1))
u = zeros((tend/tinc,1))
w = zeros((tend/tinc,1))
Vdot = zeros((tend/tinc,1))

for i in range(0,int(tend/tinc)):
    u[i] = gamma * y[i,0] * cos(y[i,1])
    w[i] = k * y[i,1] + gamma * cos(y[i,1]) * sin(y[i,1]) / y[i,1] * (y[i,1] + h * y[i,2])
    Vdot = -gamma * y[i,0]**2 * (cos(y[i,1]))**2 - k * y[i,1]**2
    xpos[i] = xend - y[i,0] * cos(y[i,2])
    ypos[i] = yend - y[i,0] * sin(y[i,2])

# Plot the trajectory to go from the start to the goal.
figure(1)
lpos = plot(xpos, ypos, 'b.')
lposStart = plot(xstart, ystart, 'go')
lposEnd = plot(xend, yend, 'ro')
title('Trajectory')
xlabel('x (m)')
ylabel('y (m)')
legend((lpos, lposStart, lposEnd), ('Position', 'Start', 'End'), 'best')
axis('equal')
show()
if saveimages:
    savefig("images/lyapunovTrajectory.png", dpi=pngres)

# Plot the X and Y positions versus time.
figure(2)
lpos = plot(t, xpos, t, ypos)
title('Position')
xlabel('Time (s)')
ylabel('Position (m)')
legend((lpos), (r'$\Delta x$', r'$\Delta y$'), 'best')
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovXYpositions.png", dpi=pngres)

# Plot the linear and angular velocities versus time.
figure(3)
lvel = plot(t, u, t, w)
title('Linear and Angular Velocities')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
legend((lvel), ('u', r'$\omega$'), 'best')
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovVelocities.png", dpi=pngres)
