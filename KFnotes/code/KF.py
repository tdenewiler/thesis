#! /usr/bin/env python

# Kalman filter example program for differential drive robots.
# Perfect measurements are available for all the states except for yaw.
# Yaw has two sensors that have adjustable amounts of noise and drift.
# Output is RMS errors for position and heading plus plots showing some of the state variables.

# The code to generate a movie with mencoder was borrowed from
# Josh Lifton 2004; http://web.media.mit.edu/~lifton/snippets/graph_movie/

# For linear algebra and plotting:
from pylab import *
import numpy
import pylab

# For movie:
import subprocess
import os
import sys

pngres = 600
showplots = 0
saveplots = 0
makemovie = 1

# Number of time steps.
N = 50

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
    x[i,6] = math.atan2(x[i,1],x[i,0])
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
Q = 0.1*numpy.eye(8)

# Measurement covariance matrix.
R = numpy.zeros((9,9))
R[6,6] = Yaw1Noise
R[7,7] = Yaw2Noise

# Initialization of state estimate and filter covariance.
xh = x[0,]
P = Q
xhat = xh
Pmp = diag(P)
Pp = diag(P)
Kk = numpy.zeros((8*(N-1),9))

# Variables for making movie of yaw estimate distribution.
xmovie = numpy.arange(-pi, pi, 0.001)
ymovieP = numpy.zeros((N, len(xmovie)), float)
ymovieM = numpy.zeros((N, len(xmovie)), float)
ymoviePmean = numpy.zeros((N), float)
ymovieMmean = numpy.zeros((N), float)

# Run the KF equations.
for i in range(N):
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
    xh = dot(F,xh) # estimate
    Pm = dot(F,dot(P,F.T))+Q # estimate covariance
    if makemovie:
        ymovieP[i] = (1/numpy.sqrt(2*numpy.pi*Pm[6,6]))*numpy.exp(-((xmovie-xh[6])**2)/(2*Pm[6,6]))
        ymoviePmean[i] = xh[6]

    # Measurement update step.
    K = dot(Pm,dot(H.T,numpy.linalg.inv(dot(H,dot(Pm,H.T))+R))) # Kalman gain
    xh = dot((eye(8)-dot(K,H)),xh) + dot(K,y[i+1,]) # estimate
    P = dot((eye(8)-dot(K,H)),Pm) # estimate covariance
    if makemovie:
        ymovieM[i] = (1/numpy.sqrt(2*numpy.pi*P[6,6]))*numpy.exp(-((xmovie-xh[6])**2)/(2*P[6,6]))
        ymovieMmean[i] = xh[6]

    # Save state estimate, state covariance and gains.
    xhat = vstack((xhat,xh))
    Pmp = vstack((Pmp,diag(Pm)))
    Pp = vstack((Pp,diag(P)))
    Kk = vstack((Kk,K))
    
if showplots or saveplots:
    # Plot the yaw estimate.
    figure(1)
    lyaw =  plot(x[:,6])
    lpsi1 = plot(psi1)
    lpsi2 = plot(psi2)
    lxhat = plot(xhat[:,6], 'kx')
    title('Yaw')
    xlabel('Time (s)')
    ylabel('Yaw (radians)')
    legend((lyaw, lpsi1, lpsi2, lxhat), ('Ground Truth', 'Sensor 1', 'Sensor 2', 'KF'))
    axis('equal')
    if saveplots:
        savefig("../images/kfSimYaw.png", dpi=pngres)
    
    # Plot the yaw estimate zoomed in.
    figure(2)
    lyaw = plot(x[:,6])
    lpsi1 = plot(psi1)
    lpsi2 = plot(psi2)
    lxhat = plot(xhat[:,6], 'kx')
    title('Yaw')
    xlabel('Time (s)')
    ylabel('Yaw (radians)')
    legend((lyaw, lpsi1, lpsi2, lxhat), ('Ground Truth', 'Sensor 1', 'Sensor 2', 'KF'))
    axis([10.5, 16, 0, 3])
    if saveplots:
        savefig("../images/kfSimYawZoom.png", dpi=pngres)
    
    # Plot the actual and estimated position.
    figure(3)
    lpos = plot(x[:,0], x[:,1])
    lposhat = plot(xhat[:,0], xhat[:,1], 'kx')
    title('Position')
    xlabel('X (m)')
    ylabel('Y (m)')
    legend((lpos, lposhat), ('Ground Truth','KF'))
    axis('equal')
    if saveplots:
        savefig("../images/kfSimPosition.png", dpi=pngres)
    if showplots:
        show()

# Calculate the RMS errors.
epos = 0
eyaw = 0
for i in range (1,N):
    epos = epos + (x[i,0]-xhat[i,0])**2 + (x[i,1]-xhat[i,1])**2
    eyaw = eyaw + (x[i,6]-xhat[i,6])**2

epos = sqrt(epos)/N
eyaw = sqrt(eyaw)/N
print 'RMS position error = ', epos, 'meters'
print 'Yaw sensor noise 1 =', Yaw1Noise*180/pi, 'degrees, 2 =', Yaw2Noise*180/pi, 'degrees'
print 'Yaw state estimate variance = ', P[6,6]*180/pi, 'degrees'
print 'RMS heading error = ', eyaw*180/pi, 'degrees'
print '2-sigma (95.4%) for sensor 1 =', 2*sqrt(Yaw1Noise*180/pi),', for estimate =', 2*sqrt(P[6,6]*180/pi)
print '3-sigma (99.7%) for sensor 1 =', 3*sqrt(Yaw1Noise*180/pi),', for estimate =', 3*sqrt(P[6,6]*180/pi)

# Try to make a movie of the normal distribution of the estimate through time with xhat and Pmp.
if makemovie:
    # Print the version information for the machine, OS,
    # Python interpreter, and matplotlib.  The version of
    # Mencoder is printed when it is called.
    print 'Executing on', os.uname()
    print 'Python version', sys.version
    
    not_found_msg = """
    The mencoder command was not found;
    mencoder is used by this script to make an avi file from a set of pngs.
    It is typically not installed by default on Linux distros because of
    legal restrictions, but it is widely available.
    """
    
    try:
        subprocess.check_call(['mencoder'])
    except subprocess.CalledProcessError:
        print "mencoder command was found"
        pass # mencoder is found, but returns non-zero exit as expected.
        # This is a quick and dirty check; it leaves some spurious output for the user to puzzle over.
    except OSError:
        print not_found_msg
        sys.exit("This part of the script can be turned off by setting makemovie=0. Quitting.\n")
    
    yaxisheight = 6
    for i in range(N*2):
        if i % 2: # odd
            lP = plot(xmovie,ymovieP[i/2],'b')
            lM = plot(0,pi,'r') # A single point to make lM show up in legend.
            axvline(ymoviePmean[i/2], 0, yaxisheight, 'g')
        else: # even
            lM = plot(xmovie,ymovieM[i/2],'r')
            lP = plot(0,pi,'b') # A single point to make lP show up in legend.
            axvline(ymovieMmean[i/2], 0, yaxisheight, 'g')
        axis((xmovie[0],xmovie[-1],-0.25,yaxisheight))
        xlabel('Yaw Angle (radians)')
        ylabel(r'Covariance (radians$.^2$)')
        title(r'Evolution of Yaw Estimate = $\cal{N}(\mu, \sigma^2)$', fontsize=20)
        legend((lP,lM),('Prediction Update','Measurement Update'))
        
        # Save each image.
        filename = str('pngtmp/%05d' % i) + '.png'
        savefig(filename, dpi=100)
    
        # Let the user know what's happening occasionally.
        if not (i+1) % 25:
            print 'Wrote file', i+1, 'of', len(ymovieP)*2, ' =', filename
    
        # Clear the figure to make way for the next image.
        clf()

    # Now that we have graphed images of the dataset, we will stitch them
    # together using Mencoder to create a movie.  Each image will become
    # a single frame in the movie.
    #
    # We want to use Python to make what would normally be a command line
    # call to Mencoder.  Specifically, the command line call we want to
    # emulate is (without the initial '#'):
    # mencoder mf://*.png -mf type=png:w=800:h=600:fps=25 -ovc lavc -lavcopts vcodec=mpeg4 -oac copy -o output.avi
    # See the MPlayer and Mencoder documentation for details.
    command = ('mencoder',
               'mf://pngtmp/*.png',
               '-mf',
               'type=png:w=800:h=600:fps=2',
               '-ovc',
               'lavc',
               '-lavcopts',
               'vcodec=mpeg4',
               '-oac',
               'copy',
               '-o',
               'kfdistribution.avi')
    
    print "\nabout to execute:\n%s\n" % ' '.join(command)
    subprocess.check_call(command)
    print "\n The movie was written to 'kfdistribution.avi'"
    print "You may want to delete pngtmp/*.png now.\n"
