#! /usr/bin/env python
"""Plot time response."""

import numpy
from scipy import * # pylint: disable=import-error,wildcard-import
import pylab # pylint: disable=import-error

# pylint: disable=invalid-name

tend = 15
tinc = 0.01
N = tend/tinc
t = numpy.arange(0, tend, tinc)

gamma = 0.2
sigma = 0.21

alpha = numpy.zeros((N), float)
de = numpy.zeros((N), float)
for i in range(N):
    alpha = de**(-1*sigma*t)
    de = de**(-1*gamma*t)

la = pylab.plot(t, alpha)
le = pylab.plot(t, de)
pylab.show()
