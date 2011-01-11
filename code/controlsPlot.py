#! /usr/bin/env python
from pylab import *
import pylab
import csv
import sys

# Open the log file and set up a reader to parse the CSV file.
f = open(sys.argv[1], "rU")
reader = csv.DictReader(f)

# Resolution of plots and whether to save them to a file.
pngres = 600
saveplots = 1

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
crossTrackError = []

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
	crossTrackError.append(float(data["crossTrackError"]))

# Find the time, average cross track error, max velocity and number of times under a minimum velocity.
x = 0
velmin = 10
atvelmin = 0
vel = u
t0 = time[0].split(":")
t1 = time[len(time)-1].split(":")
print 'Elapsed time =', (float(t1[0])-float(t0[0]))*360 + (float(t1[1])-float(t0[1]))*60 + (float(t1[2])-float(t0[2]))
for i in arange(len(crossTrackError)):
    x += crossTrackError[i]
print 'Average cross track error =', x / len(crossTrackError)
print 'Maximum velocity =', max(vel)
x = 0
for i in arange(len(vel)):
    if (vel[i] < velmin) and not (atvelmin):
        atvelmin = 1
        x += 1
    if (vel[i] > velmin) and (atvelmin):
        atvelmin = 0
print 'Number of stops =', x-1
x = 0
for i in arange(len(u)):
    x += fabs(u[i]) + fabs(w[i])
print 'Effort =', x
sys.exit()

# Plot the Lyapunov vectors versus time.
figure()
#le = plot(e)
#lheading = plot(heading)
#lgoalyaw = plot(goalyaw)
lpidu = plot(pidu)
lpidw = plot(pidw)
#lVdot = plot(Vdot)
#lu = plot(u)
#lw = plot(w)
#lencvelt = plot(encvelt)
#lencvelr = plot(encvelr)
#lvel = plot(vel)
#lyawrate = plot(yawrate)
#lyaw = plot(yaw)
#lcompassYaw = plot(compassYaw)
title("PackBot Data vs. Time")
ylim(-100,100)
xlabel("Time (1/10th s)")
ylabel("Data")

# The legend contains Unicode characters for Greek symbols.
#figlegend((lyaw, lcompassYaw), ('KF Yaw', 'Compass Yaw'), 'upper right')
#figlegend((lheading, lyaw), ('Heading', 'Yaw'), 'upper right')
#figlegend((lencvelt, lvel, lyaw), ('Encoder Velocity', 'GPS Velocity', 'Yaw'), 'upper right')
#figlegend((le, lheading, lgoalyaw, lVdot, lu, lw, lencvelt, lencvelr), ('e (m)', u"\u03b1"' (rad)', u"\u03b8"' (rad)', 'Vdot', 'u', 'w', 'Enc u', 'Enc w'), 'upper right')
#figlegend((le, lheading, lgoalyaw, lu, lw, lencvelt, lencvelr), ('e (m)', u"\u03b1"' (rad)', u"\u03b8"' (rad)', 'u', 'w', 'Enc u', 'Enc w'), 'upper right')
#figlegend((lu, lw), ('Linear Velocity (%)', 'Angular Velocity (%)'), 'upper right')
figlegend((lpidu, lpidw), ('Linear Velocity (%)', 'Angular Velocity (%)'), 'upper right')
if saveplots:
    savefig("pbData.svg")
    savefig("pbData.png", dpi = pngres)

# Plot the errors versus time.
figure()
le = plot(e)
lalpha = plot(alpha)
ltheta = plot(theta)
title("Packbot Errors vs. Time")
xlabel("Time (1/10th s)")
ylabel("Data")
figlegend((le, lalpha, ltheta), ('e', r'$\alpha$', r'$\theta$'), 'upper right')
if saveplots:
    savefig("pbDataErrors.svg")
    savefig("pbDataErrors.png", dpi = pngres)

# Display the figures and close out the program.
show()
close()
