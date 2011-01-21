#! /usr/bin/env python

# Simulation of differential drive robot using model-based controller
# based on Lyapunov stability theory.

from scipy import *
from scipy.integrate import odeint
from matplotlib import *
from pylab import *
from math import *

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
    adot = -k * y[1] - gamma * h * cos(y[1]) * sin(y[1]) / y[1]
    tdot = gamma * cos(y[1]) * sin(y[1])
    
    return [edot, adot, tdot]
# end kinematicsODE()


#####################################################################
# Main.

# Start pose.
xstart = 7
ystart = 10
yawstart = 0
xend = 0
yend = 0
yawend = 0
plotpad = 1.1

# Time to run simulation.
tend = 20
tinc = 0.1
t = arange(0, tend, tinc)

# Gains.
gamma = 0.3#0.23
h = 1.2#0.23
k = 1.2#1.38

# Calculate the initial errors for position and heading.
e = sqrt((xstart-xend)**2 + (ystart-yend)**2)
theta = atan2(yend-ystart, xend-xstart)
alpha = theta - (yawstart-yawend)
y0 = [e, theta, alpha]

# Simulate the kinematics for the trajectory to get the state variables
# through time. The state variables are:
# y[1] = e, y[2] = alpha and y[3] = theta.
y = odeint(kinematicsODE, y0, t, (gamma, h, k))

# At each time step calculate the linear and angular velocity commands.
# Note that Vdot should be <= 0 is a sanity check.
xpos = zeros((tend/tinc,1))
ypos = zeros((tend/tinc,1))
u = zeros((tend/tinc,1))
w = zeros((tend/tinc,1))
Vdot = zeros((tend/tinc,1))

for i in range(0,tend/tinc):
    xpos[i] = sqrt(y[i,0]**2 * cos(y[i,2])**2)
    ypos[i] = xpos[i] * tan(y[i,2])
    u[i] = gamma * cos(y[i,1]) * y[i,0]
    w[i] = k * y[i,1] + gamma * cos(y[i,1]) * sin(y[i,1]) / y[i,1] * (y[i,1] + h * y[i,2])
    Vdot = -y[i,0] * u * cos(y[i,1]) + y[i,1] * (-w[i] + u[i] * sin(y[i,1]) / y[i,1] * (y[i,1] + h * theta) / y[i,0])

# Plot the trajectory to go from the start to the goal.
figure(1)
lpos = plot(xpos, ypos, 'b.')
lposStart = plot(xstart, ystart, 'go')
lposEnd = plot(xend, yend, 'ro')
title('Trajectory')
xlabel('X (m)')
ylabel('Y (m)')
legend((lpos, lposStart, lposEnd), ('Position', 'Start', 'End'))
axis('equal')
#show()
if saveimages:
    savefig("images/lyapunovTrajectory.png", dpi=pngres)

# Plot the X and Y positions versus time.
figure(2)
lpos = plot(t, xpos, t, ypos)
title('Position')
xlabel('Time (s)')
ylabel('Position (m)')
legend((lpos), (r'$\Delta X$', r'$\Delta Y$'))
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
legend((lvel), ('u', r'$\omega$'))
axis('equal')
show()
if saveimages:
    savefig("images/lyapunovVelocities.png", dpi=pngres)
