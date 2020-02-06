#! /usr/bin/env python3
"""Plot controls output."""

from __future__ import print_function

import csv
import sys
import matplotlib.pyplot as plt

# pylint: disable=invalid-name

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

# Read and store data from log file in vectors using labels on first line of
# log file.
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

# Find the time, average cross track error, max velocity and number of times
# under a minimum velocity.
x = 0
velmin = 10
atvelmin = 0
vel = u
t0 = time[0].split(":")
t1 = time[len(time)-1].split(":")
hours = float(t1[0]) - float(t0[0])
minutes = float(t1[1]) - float(t0[1])
seconds = float(t1[2]) - float(t0[2])
elapsed_time = hours * 360 + minutes * 60 + seconds
print('Elapsed time = {}'.format(elapsed_time))
for i in enumerate(crossTrackError):
    x += crossTrackError[i]
print('Average cross track error = {}'.format(x / len(crossTrackError)))
print('Maximum velocity = {}'.format(max(vel)))
x = 0
for i in enumerate(vel):
    if vel[i] < velmin and not atvelmin:
        atvelmin = 1
        x += 1
    if (vel[i] > velmin) and (atvelmin):
        atvelmin = 0
print('Number of stops = {}'.format(x - 1))
x = 0
for i in enumerate(u):
    x += abs(u[i]) + abs(w[i])
print('Effort = {}'.format(x))
sys.exit()

# Plot the Lyapunov vectors versus time.
plt.figure()
# le = plot(e)
# lheading = plt.plot(heading)
# lgoalyaw = plt.plot(goalyaw)
lpidu = plt.plot(pidu)
lpidw = plt.plot(pidw)
# lVdot = plt.plot(Vdot)
# lu = plt.plot(u)
# lw = plt.plot(w)
# lencvelt = plt.plot(encvelt)
# lencvelr = plt.plot(encvelr)
# lvel = plt.plot(vel)
# lyawrate = plt.plot(yawrate)
# lyaw = plt.plot(yaw)
# lcompassYaw = plt.plot(compassYaw)
plt.title("PackBot Data vs. Time")
plt.ylim(-100, 100)
plt.xlabel("Time (1/10th s)")
plt.ylabel("Data")

# The legend contains Unicode characters for Greek symbols.
# plt.figlegend((lyaw, lcompassYaw), ('KF Yaw', 'Compass Yaw'), 'upper right')
# plt.figlegend((lheading, lyaw), ('Heading', 'Yaw'), 'upper right')
# plt.figlegend((lencvelt, lvel, lyaw),
#               ('Encoder Velocity', 'GPS Velocity', 'Yaw'), 'upper right')
# plt.figlegend((le, lheading, lgoalyaw, lVdot, lu, lw, lencvelt, lencvelr),
#               ('e (m)', u"\u03b1"' (rad)', u"\u03b8"' (rad)', 'Vdot',
#               'u', 'w', 'Enc u', 'Enc w'), 'upper right')
# plt.figlegend((le, lheading, lgoalyaw, lu, lw, lencvelt, lencvelr),
#               ('e (m)', u"\u03b1"' (rad)', u"\u03b8"' (rad)', 'u', 'w',
#               'Enc u', 'Enc w'), 'upper right')
# plt.figlegend((lu, lw), ('Linear Velocity (%)',
#               'Angular Velocity (%)'), 'upper right')
plt.figlegend((lpidu, lpidw), ('Linear Velocity (%)', 'Angular Velocity (%)'),
              'upper right')
if saveplots:
    plt.savefig("pbData.svg")
    plt.savefig("pbData.png", dpi=pngres)

# Plot the errors versus time.
plt.figure()
le = plt.plot(e)
lalpha = plt.plot(alpha)
ltheta = plt.plot(theta)
plt.title("Packbot Errors vs. Time")
plt.xlabel("Time (1/10th s)")
plt.ylabel("Data")
plt.figlegend((le, lalpha, ltheta), ('e', r'$\alpha$', r'$\theta$'),
              'upper right')
if saveplots:
    plt.savefig("pbDataErrors.svg")
    plt.savefig("pbDataErrors.png", dpi=pngres)

# Display the figures and close out the program.
plt.show()
plt.close()
