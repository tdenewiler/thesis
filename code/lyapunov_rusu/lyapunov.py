#! /usr/bin/env python

"""
Robot controller using Lyapunov control function.

Simulation of differential drive robot using model-based controller
based on Lyapunov stability theory.
"""

from __future__ import print_function

import time
from math import cos, sin, atan2, sqrt
import numpy as np
import matplotlib.pyplot as plt  # pylint: disable=import-error
from scipy.integrate import odeint  # pylint: disable=import-error

# pylint: disable=invalid-name


class Lyapunov(object):  # pylint: disable=too-many-instance-attributes, useless-object-inheritance
    """Robot controller using Lyapunov control function."""

    def __init__(self):
        """Set simulation parameters."""
        self.pngres = 100
        self.save_images = False

        # Start pose.
        self.xi = 5
        self.yi = -2
        self.yawi = 0
        self.xf = -15
        self.yf = 13
        self.yawf = 0

        # Time and resolution of simulation.
        self.tend = 40
        self.tinc = 0.1
        self.t = np.arange(0, self.tend, self.tinc)
        self.steps = int(self.tend / self.tinc)

        # Gains.
        #self.gamma = 0.25
        #self.h = 0.33
        #self.k = 0.30
        self.gamma = 0.25
        self.h = 1.2
        self.k = 2.5
        self.gamma2 = 0.25
        self.h2 = 1.10
        self.k2 = 2 * self.gamma2 * sqrt(self.h2)

        xpos, ypos, u, w, xpos2, ypos2 = self.simulate()
        self.make_plots(xpos, ypos, u, w, xpos2, ypos2)

    @classmethod
    def kinematics_ode(cls, y_ode, _, gamma_ode, h_ode, k_ode):
        r"""
        Generate trajectory.

        Solves the differential equations from robot kinematics equations
        to generate a trajectory that the robot will drive.

        ODEs
        \dot{e} = -\gamma e \cos^2\alpha
        \dot{\alpha} = -k\alpha - \gamma h \frac{\cos\alpha\sin\alpha}{\alpha}
        \dot{\theta} = \gamma\cos\alpha\sin\alpha

        Variables:
        y(0) = e
        y(1) = alpha
        y(2) = theta
        """
        edot = -gamma_ode * y_ode[0] * cos(y_ode[1])**2
        alphadot = -k_ode * y_ode[1] - gamma_ode * h_ode * y_ode[2] * \
                   cos(y_ode[1]) * sin(y_ode[1]) / y_ode[1]
        thetadot = gamma_ode * cos(y_ode[1]) * sin(y_ode[1])

        return [edot, alphadot, thetadot]

    def simulate(self):  # pylint: disable=too-many-locals
        """
        Simulate the robot kinematics to determine the trajectory of the robot.

        The state variables are: y[1] = e, y[2] = alpha and y[3] = theta.
        """
        # Calculate the initial errors for position and heading.
        dxe = self.xf - self.xi
        dye = self.yf - self.yi
        thetastar = self.yawf
        psi = self.yawi
        e = sqrt((dxe)**2 + (dye)**2)
        thetae = atan2(dye, dxe)
        phi = thetastar - psi
        theta = thetae - thetastar
        alpha = theta - phi
        y0 = [e, theta, alpha]
        print('Initial errors are e = {:.3f} m, alpha = {:.3f} rad, theta = {:.3f} rad, thetae = '
              '{:.3f} rad.'.format(e, alpha, theta, thetae))

        start = time.time()
        y = odeint(self.kinematics_ode, y0, self.t, (self.gamma, self.h, self.k))
        y2 = odeint(self.kinematics_ode, y0, self.t, (self.gamma2, self.h2, self.k2))

        # At each time step calculate the linear and angular velocity commands and the
        # resulting robot positions.
        xpos = np.zeros((self.steps, 1))
        ypos = np.zeros((self.steps, 1))
        u = np.zeros((self.steps, 1))
        w = np.zeros((self.steps, 1))
        xpos2 = np.zeros((self.steps, 1))
        ypos2 = np.zeros((self.steps, 1))
        u2 = np.zeros((self.steps, 1))
        w2 = np.zeros((self.steps, 1))

        for i in range(0, self.steps):
            u[i] = self.gamma * y[i, 0] * cos(y[i, 1])
            w[i] = self.k * y[i, 1] + self.gamma * cos(y[i, 1]) * sin(y[i, 1]) / y[i, 1] * \
                   (y[i, 1] + self.h * y[i, 2])
            xpos[i] = self.xf - y[i, 0] * cos(y[i, 2])
            ypos[i] = self.yf - y[i, 0] * sin(y[i, 2])
            u2[i] = self.gamma * y2[i, 0] * cos(y2[i, 1])
            w2[i] = self.k * y2[i, 1] + self.gamma * cos(y2[i, 1]) * sin(y2[i, 1]) / y2[i, 1] * \
                    (y2[i, 1] + self.h * y2[i, 2])
            xpos2[i] = self.xf - y2[i, 0] * cos(y2[i, 2])
            ypos2[i] = self.yf - y2[i, 0] * sin(y2[i, 2])
        elapsed = time.time() - start
        print('Simulation took {:.5f} seconds for a rate of {:.2f} Hz.'.format(elapsed, 1/elapsed))

        return xpos, ypos, u, w, xpos2, ypos2

    def make_plots(self, xpos, ypos, u, w, xpos2, ypos2):  # pylint: disable=too-many-statements, too-many-arguments, too-many-locals
        """Plot the trajectory to go from the start to the goal."""
        fignum = 1
        plt.figure(fignum)
        plt.plot(xpos, ypos, 'b.', label='Position')
        plt.plot(self.xi, self.yi, 'go', label='Start')
        plt.plot(self.xf, self.yf, 'ro', label='End')
        plt.title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' %
                  (self.gamma, self.h, self.k))
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.axis('equal')
        #plt.show()
        if self.save_images:
            plt.savefig("images/lyapunovTrajectory.png", dpi=self.pngres)

        # Plot the trajectory to go from the start to the goal.
        fignum = fignum + 1
        plt.figure(fignum)
        plt.plot(xpos2, ypos2, 'b.', label='Position')
        plt.plot(self.xi, self.yi, 'go', label='Start')
        plt.plot(self.xf, self.yf, 'ro', label='End')
        plt.title(r'Robot Trajectory ($\gamma$ = %0.2f, h = %0.2f, k = %0.2f)' %
                  (self.gamma2, self.h2, self.k2))
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.legend()
        plt.axis('equal')
        plt.show()
        if self.save_images:
            plt.savefig("images/lyapunovTrajectory2.png", dpi=self.pngres)

        # Plot the X and Y position errors versus time.
        xdiff = np.zeros((self.steps, 1))
        ydiff = np.zeros((self.steps, 1))
        for i in range(0, self.steps):
            xdiff[i] = self.xf - xpos[i]
            ydiff[i] = self.yf - ypos[i]
        fignum = fignum + 1
        plt.figure(fignum)
        plt.plot(self.t, xdiff, label=r'$\Delta$ x')
        plt.plot(self.t, ydiff, label=r'$\Delta$ y')
        plt.title('Position Errors')
        plt.xlabel('Time (s)')
        plt.ylabel('Position Error (m)')
        plt.legend()
        plt.axis('equal')
        #plt.show()
        if self.save_images:
            plt.savefig("images/lyapunovXYpositions.png", dpi=self.pngres)

        # Plot the linear and angular velocities versus time.
        fignum = fignum + 1
        plt.figure(fignum)
        plt.plot(self.t, u, label='u (m/s)')
        plt.plot(self.t, w, label=r'$\omega$ (rad/s)')
        plt.title('Linear and Angular Velocities')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity')
        plt.legend()
        plt.axis('equal')
        plt.show()
        if self.save_images:
            plt.savefig("images/lyapunovVelocities.png", dpi=self.pngres)


if __name__ == "__main__":
    Lyapunov()
