#! /usr/bin/env python

# For parsing log files and plotting:
from pylab import *
import pylab
import csv
from matplotlib import *
import pygame

# For movie:
import subprocess
import os
import sys

# Open the log file and set up a reader to parse the CSV file.
f = open(sys.argv[1], "rU")
reader = csv.DictReader(f)

# Resolution of plots.
# 200 dpi is great for movies, 100 is okay but may be blurry on large projectors.
pngres = 100

# Declare vectors to hold data.
date = []
time = []
u = []
w = []
pidu = []
pidw = []
h = []
k = []
gamma = []
heading = []
goalyaw = []
e = []
Vdot = []
dx = []
dy = []
alpha = []
theta = []
encvelt = []
encvelr = []
vel = []
yawrate = []
yaw = []
compassYaw = []

# Read and store data from log file in vectors using labels on first line of log file.
for data in reader:
	date.append(str(data["date"]))
	time.append(str(data["time"]))
	u.append(float(data["u"]))
	w.append(float(data["w"]))
	pidu.append(float(data["pidu"]))
	pidw.append(float(data["pidw"]))
	h.append(float(data["h"]))
	k.append(float(data["k"]))
	gamma.append(float(data["gamma"]))
	heading.append(float(data["heading"]))
	goalyaw.append(float(data["goalyaw"]))
	e.append(float(data["e"]))
	Vdot.append(float(data["Vdot"]))
	dx.append(float(data["dx"]))
	dy.append(float(data["dy"]))
	alpha.append(float(data["alpha"]))
	theta.append(float(data["theta"]))
	encvelt.append(float(data["encvelt"]))
	encvelr.append(float(data["encvelr"]))
	vel.append(float(data["vel"]))
	yawrate.append(float(data["yawrate"]))
	yaw.append(float(data["yaw"]))
	compassYaw.append(float(data["compassYaw"]))

#print 'e has', len(e), 'elements'
#le = plot(e[0:500])
#title("Controller Errors vs. Time")
#xlabel("Time")
#ylabel("Errors")
#figlegend((le), ('e'), 'upper right')
#show()
#close()
#sys.exit("This part of the script can be turned off by setting makemovie=0. Quitting.\n")

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
    
# Go through each data set and generate a new plot.
N = len(time)
#N = 1000
print 'Image resolution is', pngres, 'dpi'
print 'Using', N, 'samples.'
titlename = str('Controller Errors with h = %.2f' % h[0]) + str(', $\gamma$ = %.2f' % gamma[0]) + str(', k = %.2f' %k[0])
print titlename
for i in range(N):
    le = plot(e[0:i])
    lalpha = plot(alpha[0:i])
    ltheta = plot(theta[0:i])
    title(titlename)
    xlabel("Time")
    ylabel("Errors")
    figlegend((le,lalpha,ltheta), ('e',r'$\alpha$',r'$\theta$'), 'upper right')
    # Save each image.
    filename = str('pngtmp/%05d' % i) + '.png'
    savefig(filename, dpi=pngres)
    # Let the user know what's happening occasionally.
    if not (i+1) % 25:
        print 'Wrote file', i+1, 'of', N, ' =', filename
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
           'type=png:w=800:h=600:fps=10', #fps = 10 seems about real-time since log file is 10Hz
           '-ovc',
           'lavc',
           '-lavcopts',
           'vcodec=mpeg4',
           '-oac',
           'copy',
           '-o',
           'controllerErrors.avi')

print "\nabout to execute:\n%s\n" % ' '.join(command)
subprocess.check_call(command)
print "\n The movie was written to 'controllerErrors.avi'"
print "You may want to delete pngtmp/*.png now.\n"
