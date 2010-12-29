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
showplots = 1

# Number of time steps.
N = 50

# The state vector is x = [X Y Z V theta(pitch) phi(roll) psi(yaw) w(yaw rate)].
x = numpy.zeros((8,1))

# Set up the measurements.
y = numpy.zeros((3,1))
y[0] = (360./(N))*pi/180 # yaw rate
y[1] = 0*pi/180 # pitch
y[2] = 0*pi/180 # roll

# Measurement transition matrix. #cols = #states, #rows = #measurements.
H = numpy.zeros((3,8))
# Set up H to reflect that yaw rate, pitch and roll are measured.
H[0,7] = 1 # yaw rate
H[1,4] = 1 # pitch
H[2,5] = 1 # roll

# State covariance matrix.
Q = 0.1*numpy.eye(8)

# Measurement covariance matrix.
R = numpy.zeros((3,3)) # Zero noise for the sensors.

# Initialization of state estimate and filter covariance.
P = Q
I = numpy.eye(8)

# Save the yawstate values for plotting.
n = 30
yaw = numpy.zeros((n,1))
pitch = numpy.zeros((n,1))

# Run the KF equations with pitch and roll non-zero.
for j in range(n):
    x = numpy.zeros((8,1))
    y[2] = 0*pi/180 # roll
    y[1] = j*pi/180 # pitch
    for i in range(N+1):
        x[7] = x[7] * cos(x[4]) / cos(x[5])
        # State transition matrix based on dynamics.
        F = array([ [0,0,0,cos(x[6])*cos(x[4]),-cos(x[6])*sin(x[4]),0,-sin(x[6])*cos(x[5]),0],
                  [  0,0,0,sin(x[6])*cos(x[4]),-sin(x[6])*sin(x[4]),0,cos(x[6])*cos(x[4]),0],
                  [  0,0,0,-sin(x[4]),-cos(x[4]),0,0,0],
                  [  0,0,0,0,0,0,0,0],
                  [  0,0,0,0,0,-x[7]*cos(x[5]),0,-sin(x[5])],
                  [  0,0,0,0,x[7]*cos(x[5])/(cos(x[4])**2),x[7]*tan(x[4])*sin(x[5]),0,tan(x[4])*cos(x[5])],
                  [  0,0,0,0,x[7]*sin(x[4])*cos(x[5])/(cos(x[4])**2),-x[7]*sin(x[5])/cos(x[4]),0,cos(x[5])/cos(x[4])],
                  [  0,0,0,0,0,0,0,0] ])
    
        Phi = array([ [1,0,0,cos(x[6])*cos(x[4]),0,0,0,0],
                    [  0,1,0,sin(x[6])*cos(x[4]),0,0,0,0],
                    [  0,0,1,-sin(x[4]),0,0,0,0],
                    [  0,0,0,1,0,0,0,0],
                    [  0,0,0,0,1,0,0,-sin(x[5])],
                    [  0,0,0,0,0,1,0,tan(x[4])*cos(x[5])],
                    [  0,0,0,0,0,0,1,cos(x[5])/cos(x[4])],
                    [  0,0,0,0,0,0,0,1] ])
    
        # Prediction update step.
        x = dot(Phi,x) # mean of estimate
        P = dot(Phi,dot(P,Phi.T))+Q # covariance of estimate
        #P = dot(F,dot(P,F.T))+Q # covariance of estimate
    
        # Measurement update step.
        K = dot(P,dot(H.T,numpy.linalg.inv(dot(H,dot(P,H.T))+R))) # Kalman gain
        x = x + dot(K,y-dot(H,x)) # mean of estimate
        P = dot((I-dot(K,H)),P) # covariance of estimate
    pitch[j] = j
    yaw[j] = x[6]*180/pi
    print 'KF: Pitch =', x[4]*180/pi, 'Roll =', x[5]*180/pi, 'Yaw Speed =', x[7]*180/pi, 'Yaw =', x[6]*180/pi

if showplots:
    figurenum = 1
    # Plot the yaw estimate.
    figure(figurenum)
    figurenum = figurenum + 1
    lyaw = plot(pitch,yaw)
    title('Yaw vs. Pitch')
    xlabel('Pitch (radians)')
    ylabel('Yaw (radians)')
    figlegend((lyaw), (u"\u0471"), 'upper right')
    #axis('equal')
    show()
    close()
