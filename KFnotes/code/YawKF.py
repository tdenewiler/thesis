#! /usr/bin/env python

# Kalman filter example program for differential drive robots.
# What effect do pitch, roll and yaw rate have on yaw state?
# Assume sensors are measuring velocity, yaw rate, pitch and roll.

# For linear algebra and plotting:
from pylab import *
import numpy
import pylab

# Plotting variables.
pngres = 600
showplots = 0

# temp stuff
#Kt = numpy.zeros((8,4))
Pt = numpy.zeros((8,8))
Ht = numpy.zeros((4,8))
Rt = numpy.zeros((4,4))

Pt[3,3] = 0.000276
Pt[4,4] = 0.000302
Pt[4,6] = -0.000001
Pt[5,5] = 0.000303
Pt[5,6] = 0.000001
Pt[6,4] = -0.000001
Pt[6,5] = 0.000001
Pt[6,6] = 0.003360
Pt[7,7] = 1.092708

Ht[0,3] = 1
Ht[1,7] = 1
Ht[2,4] = 1
Ht[3,5] = 1

Rt[1,1] = 0.000027
Rt[2,2] = 0.002742
Rt[3,3] = 0.002742

tmpHPH = dot(Ht,dot(Pt,Ht.T))
tmpHPHR = dot(Ht,dot(Pt,Ht.T)) + Rt
print 'HPH.T =\n', tmpHPH
print '\nR =\n', Rt
print '\nHPH.T + R =\n', tmpHPHR

tmpINV = numpy.linalg.inv(dot(Ht,dot(Pt,Ht.T))+Rt)
print '\n(HPH.T + R)^-1 =\n', tmpINV

tmpPH = dot(Pt,Ht.T)
print '\nPH.T =\n', tmpPH

Kt = dot(Pt,dot(Ht.T,numpy.linalg.inv(dot(Ht,dot(Pt,Ht.T))+Rt))) # Kalman gain
print '\nK =\n', Kt

# Number of time steps.
N = 50

# The state vector is x = [X Y Z V theta(pitch) phi(roll) psi(yaw) w(ang rate)]
x = numpy.zeros((8,1))

# Set up the measurements.
y = numpy.zeros((4,1))
y[0,0] = 0 # velocity
y[1,0] = 0.1*pi/180 # yaw rate
y[2,0] = 2*pi/180 # pitch
y[3,0] = 2*pi/180 # roll
print 'Velocity =', y[0,0],', yaw rate =', y[1,0],', pitch =', y[2,0],', roll = ', y[3,0]

# Measurement transition matrix. #cols = #states, #rows = #measurements.
H = numpy.zeros((4,8))
# Set up H to reflect that velocity, yaw rate, pitch and roll are measured.
H[0,3] = 1 # velocity
H[1,7] = 1 # yaw rate
H[2,4] = 1 # pitch
H[3,5] = 1 # roll

# State covariance matrix.
Q = 0.1*numpy.eye(8)

# Measurement covariance matrix.
R = numpy.zeros((4,4)) # Zero noise for the sensors.

# Initialization of state estimate and filter covariance.
P = Q
I = numpy.eye(8)

# Save the yawstate values for plotting.
yawstate = numpy.zeros((N,1))

# Run the KF equations.
for i in range(N):
    # Set the yaw rate to zero half way through the run.
    if i > N/2:
        y[1,0] = 0
    # State transition matrix based on dynamics.
    F = array([ [1,0,0,cos(x[7])*cos(x[5]),0,0,0,0],
                [0,1,0,sin(x[7])*cos(x[5]),0,0,0,0],
                [0,0,1,-sin(x[5]),0,0,0,0],
                [0,0,0,1,0,0,0,0],
                [0,0,0,0,1,0,0,-sin(x[6])],
                [0,0,0,0,0,1,0,tan(x[5])*cos(x[6])],
                [0,0,0,0,0,0,1,cos(x[6])/cos(x[5])],
                [0,0,0,0,0,0,0,1] ])

    # Prediction update step.
    x = dot(F,x) # estimate
    P = dot(F,dot(P,F.T))+Q # estimate covariance

    # Measurement update step.
    K = dot(P,dot(H.T,numpy.linalg.inv(dot(H,dot(P,H.T))+R))) # Kalman gain
    x = dot((I-dot(K,H)),x) + dot(K,y) # estimate
    P = dot((I-dot(K,H)),P) # estimate covariance
    #print 'yaw =', x[6]
    yawstate[i] = x[6]

if showplots:
    figurenum = 1
    # Plot the yaw estimate.
    figure(figurenum)
    figurenum = figurenum + 1
    lyaw = plot(yawstate)
    title('Yaw')
    xlabel('Time (s)')
    ylabel('Yaw (radians)')
    figlegend((lyaw), (u"\u0471"), 'upper right')
    #axis('equal')
    if showplots:
        show()
        close()
