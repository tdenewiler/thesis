#! /usr/bin/env python

# For linear algebra and plotting:
from scipy import *
import numpy
import pylab

tend = 15
tinc = 0.01
N = tend/tinc
t = numpy.arange(0,tend,tinc)

gamma = 0.2
sigma = 0.21

alpha = numpy.zeros((N), float)
de = numpy.zeros((N), float)
for i in range(N):
    alpha = e**(-1*sigma*t)
    de = e**(-1*gamma*t)

la = pylab.plot(t, alpha)
le = pylab.plot(t, de)
pylab.show()
