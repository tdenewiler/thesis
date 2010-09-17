#! /usr/bin/env python
from scipy import *
from pylab import *

xres = 0.001
x = arange(-pi,pi+xres,xres)
ysin = sin(x)
ycos = cos(x)
ysincos = sin(x)*cos(x)
yx0 = x*0 # Horizontal line for x-axis
yx = x*1 # To compare angle to other values
plot(x,ysin)
hold(True)
plot(x,ycos)
plot(x,ysincos)
plot(x,yx)
plot(x,yx0,'k')
xlabel('x')
ylabel('y')
title(r'Effect of $\alpha$ on $\omega$')
legend((r'$\sin(\alpha)$',r'$\cos(\alpha)$',r'$\cos(\alpha)*\sin(\alpha)$',r'$\alpha$'),'upper left')
#grid(True)

show()
close()
