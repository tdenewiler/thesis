#! /usr/bin/env python
import sys
import random
i = 1
error = 0
step = 0.00000001
dthresh = 0.0000000001

for x in range(10):
    derror=sys.maxint
    theta1 = random.random()*100
    theta2 = random.random()*100
    theta0 = random.random()*100
    while derror>dthresh:
        diff = 400 - (theta0 + 2104 * theta1 + 3 * theta2)
        theta0 = theta0 + step * diff * 1
        theta1 = theta1 + step * diff * 2104
        theta2 = theta2 + step * diff * 3
        hserror = diff**2/2
        derror = abs(error - hserror)
        error = hserror
        #print 'iteration : %d, error : %s, derror : %s' % (i, error, derror)
        i+=1
    print ' theta0 : %d, theta1 : %d, theta2 : %d' % (theta0, theta1, theta2)
    print ' done : %d' %(theta0 + 2104 * theta1 + 3*theta2)
