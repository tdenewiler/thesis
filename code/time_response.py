#! /usr/bin/env python3
"""Plot time response."""

from math import floor
import numpy
import pylab  # pylint: disable=import-error

# pylint: disable=invalid-name

tend = 15
tinc = 0.01
N = floor(tend/tinc)
t = numpy.arange(0, tend, tinc)

gamma = 0.2
sigma = 0.21

alpha = numpy.zeros(N)
de = numpy.zeros(N)
for i in range(N):
    alpha = de**(-1*sigma*t)
    de = de**(-1*gamma*t)

la = pylab.plot(t, alpha)
le = pylab.plot(t, de)
pylab.show()
