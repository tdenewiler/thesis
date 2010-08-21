#! /usr/bin/env python

# Kalman filter example program for differential drive robots.
# Perfect measurements are available for all the states except for yaw.
# Yaw has three sensors that have adjustable amounts of noise and drift.
# Output is RMS errors for position and heading plus plots showing some of the state variables.

from scipy import *
import numpy
import pylab

pngres = '-r600'
saveimages = 0

# Number of time steps.
N = 100

# Sensor behavior variables. At least one Noise value must be > 0 else the inverse in the gain
# calculation is ill-conditioned.
Yaw1Noise = 1 * pi / 180
Yaw2Noise = 5 * pi / 180
Yaw1Drift = 0
Yaw2Drift = 0

# BEGIN {Data Generation}
##################################################################################################
# This part is secret business and involves the generation of the data.
# The data arise in the real world and their generation is invisible to us.

# Set up ground truth for state variables and sensor measurements.
# The state vector is x = [X Y Z V theta(pitch) phi(roll) psi(yaw) w(ang rate)]
x = numpy.zeros((N+1,8))
y = numpy.zeros((N+1,9))
psi1 = numpy.zeros((N+1,1))
psi2 = numpy.zeros((N+1,1))
R = 100 # Path is a circle, radius R.
angle = 0
for i in range(0,N+1):
    x[i,0] = R*cos(angle)
    x[i,1] = R*sin(angle)
    x[i,2] = 0
    x[i,3] = 1
    x[i,4] = 0
    x[i,5] = 0
    x[i,6] = -1 * math.atan2(x[i,1],x[i,0])
    x[i,7] = 4*x[i,3]/R
    angle = angle + 2*pi/N
    
    # Add noise and drift to IMU yaw measurements.
    psi1[i] = x[i,6] + Yaw1Noise*randn(1) + i*Yaw1Drift
    psi2[i] = x[i,6] + Yaw2Noise*randn(1) + i*Yaw2Drift

    # Set up measurement vector with ground truth except for multiple heading measurements.
    y[i,0] = x[i,0]
    y[i,1] = x[i,1]
    y[i,2] = x[i,2]
    y[i,3] = x[i,3]
    y[i,4] = x[i,4]
    y[i,5] = x[i,5]
    y[i,6] = psi1[i]
    y[i,7] = psi2[i]
    y[i,8] = x[i,7]

# Now we have the data vector y, which is all that will be passed into the real world.
##################################################################################################
# END {Data Generation}

# Measurement transition matrix. #cols = #states, #rows = #measurements.
H1 = numpy.eye(8)
H2 = numpy.zeros((1,8))
H = vstack((H1,H2))
# Set up H to reflect that yaw (column 7) is measured three times, all others measured once.
H[6,6] = 1
H[7,6] = 1
H[8,7] = 1

# State covariance matrix.
Q = numpy.eye(8)

# Measurement covariance matrix.
R = numpy.zeros((9,9))
R[6,6] = Yaw1Noise
R[7,7] = Yaw2Noise

# Initialization of state estimate and filter covariance.
xh = x[0,]
P = Q
P[6,6] = max(Yaw1Noise,Yaw2Noise)
xhat = xh
Pp = diag(P)
Kk = numpy.zeros((8*(N-1),9))

# Run the KF equations.
for i in range(0,N):
    # State transition matrix based on dynamics.
    F = array([ [1,0,0,cos(xh[7])*cos(xh[5]),0,0,0,0],
                [0,1,0,sin(xh[7])*cos(xh[5]),0,0,0,0],
                [0,0,1,-sin(xh[5]),0,0,0,0],
                [0,0,0,1,0,0,0,0],
                [0,0,0,0,1,0,0,-sin(xh[6])],
                [0,0,0,0,0,1,0,tan(xh[5])*cos(xh[6])],
                [0,0,0,0,0,0,1,cos(xh[6])/cos(xh[5])],
                [0,0,0,0,0,0,0,1] ])

    # Prediction update step.
    xh = dot(F,xh)
    Pm = dot(F,dot(P,F.T))+Q

    # Measurement update step.
    K = dot(Pm,dot(H.T,numpy.linalg.inv(dot(H,dot(Pm,H.T))+R)))
    P = dot((eye(8)-dot(K,H)),Pm)
    xh = dot((eye(8)-dot(K,H)),xh) + dot(K,y[i+1,])

    # Save state estimate, state covariance and gains.
    xhat = vstack((xhat,xh))
    Pp = vstack((Pp,diag(P)))
    Kk = vstack((Kk,K))

# Plot the yaw estimate.
pylab.figure(1)
lyaw =  pylab.plot(x[:,6])
lpsi1 = pylab.plot(psi1)
lpsi2 = pylab.plot(psi2)
lxhat = pylab.plot(xhat[:,6])
pylab.title('Yaw')
pylab.xlabel('Time (s)')
pylab.ylabel('Yaw (radians)')
pylab.legend((lyaw, lpsi1, lpsi2, lxhat), ('Ground Truth', 'Sensor 1', 'Sensor 2', 'KF'))
pylab.axis('equal')

# Plot the yaw estimate zoomed in.
pylab.figure(2)
lyaw = pylab.plot(x[:,6])
lpsi1 = pylab.plot(psi1)
lpsi2 = pylab.plot(psi2)
lxhat = pylab.plot(xhat[:,6])
pylab.title('Yaw')
pylab.xlabel('Time (s)')
pylab.ylabel('Yaw (radians)')
pylab.legend((lyaw, lpsi1, lpsi2, lxhat), ('Ground Truth', 'Sensor 1', 'Sensor 2', 'KF'))
pylab.axis([55.5, 61, 0.5, 4])

# Plot the actual and estimated position.
pylab.figure(3)
lpos = pylab.plot(x[:,0], x[:,1])
lposhat = pylab.plot(xhat[:,0], xhat[:,1])
pylab.title('Position')
pylab.xlabel('X (m)')
pylab.ylabel('Y (m)')
pylab.legend((lpos, lposhat), ('Ground Truth','KF'))
pylab.axis('equal')
pylab.show()

# Calculate the RMS errors.
epos = 0
eyaw = 0
for i in range (1,N):
    epos = epos + (x[i,0]-xhat[i,0])**2 + (x[i,1]-xhat[i,1])**2
    eyaw = eyaw + (x[i,6]-xhat[i,6])**2

epos = sqrt(epos/N)
eyaw = sqrt(eyaw/N)
print 'RMS position error = ', epos, 'meters'
print 'RMS heading error = ', eyaw, 'radians = ', eyaw*180/pi, 'degrees'
