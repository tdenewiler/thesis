#! /usr/bin/env python

#####################################################################
# Simulation of differential drive robot using model-based controller
# based on Lyapunov stability theory.
#####################################################################

import matplotlib
# For use over SSH.
matplotlib.use('Qt4Agg')

from scipy import *
from scipy.integrate import odeint
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

############################
# Configuration starts here.

# Start pose.
xi = 5
yi = -2
yawi = 0
xf = -15
yf = 13
yawf = 0

# Time and resolution of simulation.
tend = 40
tinc = 0.1
t = arange(0, tend, tinc)

# Gains.
#gamma = 0.25
#h = 0.33
#k = 0.30
gamma = 0.25
h = 1.2
k = 2.5
gamma2 = 0.25
h2 = 1.10
k2 = 2 * gamma2 * sqrt(h2)

# End of configuration.
#######################

# Calculate the initial errors for position and heading.
dxe = xf - xi
dye = yf - yi
thetastar = yawf
psi = yawi
e = sqrt((dxe)**2 + (dye)**2)
thetae = atan2(dye, dxe)
phi = thetastar - psi
theta = thetae - thetastar
alpha = theta - phi
y0 = [e, theta, alpha]
print 'Initial errors are e = %0.5f m, alpha = %0.5f rad, theta = %0.5f rad, thetae = %0.5f rad.' % (e, alpha, theta, thetae)

# Simulate the robot kinematics to determine the trajectory of the robot.
# The state variables are: y[1] = e, y[2] = alpha and y[3] = theta.
start = time.time()
y = odeint(kinematicsODE, y0, t, (gamma, h, k))
y2 = odeint(kinematicsODE, y0, t, (gamma2, h2, k2))

# At each time step calculate the linear and angular velocity commands and the resulting robot positions.
xpos = zeros((tend/tinc,1))
ypos = zeros((tend/tinc,1))
u = zeros((tend/tinc,1))
w = zeros((tend/tinc,1))
xpos2 = zeros((tend/tinc,1))
ypos2 = zeros((tend/tinc,1))
u2 = zeros((tend/tinc,1))
w2 = zeros((tend/tinc,1))

for i in range(0,int(tend/tinc)):
    u[i] = gamma * y[i,0] * cos(y[i,1])
    w[i] = k * y[i,1] + gamma * cos(y[i,1]) * sin(y[i,1]) / y[i,1] * (y[i,1] + h * y[i,2])
    xpos[i] = xf - y[i,0] * cos(y[i,2])
    ypos[i] = yf - y[i,0] * sin(y[i,2])
    u2[i] = gamma * y2[i,0] * cos(y2[i,1])
    w2[i] = k * y2[i,1] + gamma * cos(y2[i,1]) * sin(y2[i,1]) / y2[i,1] * (y2[i,1] + h * y2[i,2])
    xpos2[i] = xf - y2[i,0] * cos(y2[i,2])
    ypos2[i] = yf - y2[i,0] * sin(y2[i,2])
elapsed = time.time() - start
print 'Simulation took %0.5f seconds for a rate of %0.2f Hz.' % (elapsed, 1/elapsed)

########
# Plots.

# Plot the trajectory to go from the start to the goal.
fignum = 1
figure(fignum)
lpos, = plot(xpos, ypos, 'b.')
lposStart, = plot(xi, yi, 'go')
lposEnd, = plot(xf, yf, 'ro')
title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' % (gamma, h, k))
xlabel('x (m)')
ylabel('y (m)')
legend((lpos, lposStart, lposEnd), ('Position', 'Start', 'End'), 'best')
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovTrajectory.png", dpi=pngres)

# Plot the trajectory to go from the start to the goal.
fignum = fignum + 1
figure(fignum)
lpos, = plot(xpos2, ypos2, 'b.')
lposStart, = plot(xi, yi, 'go')
lposEnd, = plot(xf, yf, 'ro')
title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' % (gamma2, h2, k2))
xlabel('x (m)')
ylabel('y (m)')
legend((lpos, lposStart, lposEnd), ('Position', 'Start', 'End'), 'best')
axis('equal')
show()
if saveimages:
    savefig("images/lyapunovTrajectory2.png", dpi=pngres)

# Plot the X and Y position errors versus time.
xdiff = zeros((tend/tinc,1))
ydiff = zeros((tend/tinc,1))
for i in range(0,int(tend/tinc)):
    xdiff[i] = xf - xpos[i]
    ydiff[i] = yf - ypos[i]
fignum = fignum + 1
figure(fignum)
lpos = plot(t, xdiff, t, ydiff)
title('Position Errors')
xlabel('Time (s)')
ylabel('Position Error (m)')
legend((lpos), (r'$\Delta x$', r'$\Delta y$'), 'best')
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovXYpositions.png", dpi=pngres)

# Plot the linear and angular velocities versus time.
fignum = fignum + 1
figure(fignum)
lvel = plot(t, u, t, w)
title('Linear and Angular Velocities')
xlabel('Time (s)')
ylabel('Velocity')
legend((lvel), ('u (m/s)', r'$\omega$ (rad/s)'), 'best')
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovVelocities.png", dpi=pngres)

# end lyapunov.py
#################
