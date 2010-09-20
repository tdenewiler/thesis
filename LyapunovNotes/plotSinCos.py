#! /usr/bin/env python
from scipy import *
from pylab import *

saveplots = 1
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
axis('equal')
xlabel('x')
ylabel('y')
title(r'Useful Trigonometric Functions of $\alpha$')
legend((r'$\sin(\alpha)$',r'$\cos(\alpha)$',r'$\cos(\alpha)*\sin(\alpha)$',r'$\alpha$'),'upper left')

if saveplots:
    savefig("images/plotSinCos.svg")
    savefig("images/plotSinCos.png", dpi = 300)

show()
close()
