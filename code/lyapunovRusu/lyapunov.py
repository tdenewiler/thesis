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
    adot = -k * y[1] - gamma * h * y[2] * cos(y[1]) * sin(y[1]) / y[1]
    tdot = gamma * cos(y[1]) * sin(y[1])
    u = gamma * cos(y[1]) * y[0]
    w = k * y[1] + gamma * cos(y[1]) * sin(y[1]) / y[1] * (y[1] + h * y[2])
    xdot = u * cos(w)
    ydot = u * sin(w)
    
    return [edot, adot, tdot, xdot, ydot]
# end kinematicsODE()


#####################################################################
# Main.

# Start pose.
xstart = 0
ystart = 0
yawstart = 0
xend = 1
yend = 1
yawend = 0
plotpad = 1.1

# Time to run simulation.
tend = 20
tinc = 0.1
t = arange(0, tend, tinc)

# Gains.
gamma = 0.25
h = 0.33
k = 0.30

# Calculate the initial errors for position and heading.
e = sqrt((xstart-xend)**2 + (ystart-yend)**2)
theta = atan2(yend-ystart, xend-xstart)
alpha = theta - (yawstart-yawend)
xpos = xstart
ypos = ystart
y0 = [e, theta, alpha, xpos, ypos]

# Simulate the kinematics for the trajectory to get the state variables
# through time. The state variables are:
# y[1] = e, y[2] = alpha and y[3] = theta.
start = time.time()
y = odeint(kinematicsODE, y0, t, (gamma, h, k))
elapsed = time.time() - start
print 'ODE solver took %0.5f seconds' % elapsed
print 'y = e, alpha, theta, x, y'
print y

# At each time step calculate the linear and angular velocity commands.
# Note that Vdot should be <= 0 is a sanity check.
#xpos = zeros((tend/tinc,1))
#ypos = zeros((tend/tinc,1))
#u = zeros((tend/tinc,1))
#w = zeros((tend/tinc,1))
#Vdot = zeros((tend/tinc,1))
#phi = zeros((tend/tinc,1))
#xpos[0] = xstart
#ypos[0] = ystart
#phi[0] = y[0,2] - y[0,1]

#for i in range(0,int(tend/tinc)):
#    u[i] = gamma * cos(y[i,1]) * y[i,0]
#    w[i] = k * y[i,1] + gamma * cos(y[i,1]) * sin(y[i,1]) / y[i,1] * (y[i,1] + h * y[i,2])
#    Vdot = -y[i,0] * u * cos(y[i,1]) + y[i,1] * (-w[i] + u[i] * sin(y[i,1]) / y[i,1] * (y[i,1] + h * theta) / y[i,0])
#    phi[i] = y[i,2] - y[i,1]
#    xpos[i] = sqrt(y[i,0]**2 * cos(y[i,2])**2)
#    ypos[i] = xpos[i] * tan(y[i,2])
#    #xpos[i] = u[i] * cos(phi[i])
#    #ypos[i] = u[i] * sin(phi[i])
#    #print 'e =', y[i,0], 'alpha =', y[i,1], 'theta =', y[i,2]
#print 'errors =', y[int(tend/tinc)-1]

# Plot the trajectory to go from the start to the goal.
figure(1)
lpos = plot(xpos, ypos, 'b.')
lposStart = plot(xstart, ystart, 'go')
lposEnd = plot(xend, yend, 'ro')
title('Trajectory')
xlabel('X (m)')
ylabel('Y (m)')
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
legend((lpos), (r'$\Delta X$', r'$\Delta Y$'), 'best')
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
